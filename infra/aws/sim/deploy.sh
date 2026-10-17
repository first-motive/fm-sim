#!/usr/bin/env bash
# deploy.sh — operator front door for the AWS sim host stack (host.yaml).
#
# Nothing here touches AWS unless an operator names a verb that must:
#
#   ./deploy.sh validate              cfn-lint, unit tests, shellcheck; local only
#   ./deploy.sh package               build the controller ZIP, print its SHA-256; local only
#   ./deploy.sh plan                  print the upload and deploy commands; local only
#   ./deploy.sh deploy                same as plan
#   ./deploy.sh deploy --execute      upload the controller and deploy the stack
#   ./deploy.sh status                read the published status (read-only AWS call)
#   ./deploy.sh start | stop          ask the controller to start or stop the host
#
# Config (environment):
#   FM_AWS_PROFILE      AWS CLI profile                       (required for AWS verbs)
#   FM_AWS_REGION       region                                (required for AWS verbs)
#   FM_SIM_STACK        stack name                            (default: fm-sim-cloud-host)
#   FM_SIM_PARAMS       KEY=VALUE file of stack parameters    (required for plan/deploy)
#   FM_SIM_CODE_BUCKET  operator-owned bucket for the ZIP     (required for plan/deploy)
#   FM_SIM_BUILD_DIR    where package writes the ZIP          (default: a temp dir)
#
# The params file stays on the operator's machine; host.params.example lists the
# keys. It never holds a secret: the Tailscale key is named, not included.

set -euo pipefail

here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stack="${FM_SIM_STACK:-fm-sim-cloud-host}"
template="$here/host.yaml"

die() { printf 'deploy.sh: %s\n' "$*" >&2; exit 1; }

need() {
  local name
  for name in "$@"; do
    [ -n "${!name:-}" ] || die "$name is not set"
  done
}

aws_cli() {
  need FM_AWS_PROFILE FM_AWS_REGION
  aws --profile "$FM_AWS_PROFILE" --region "$FM_AWS_REGION" "$@"
}

validate() {
  uvx cfn-lint==1.57.2 "$template"
  uv run --no-project --with pytest==8.4.2 --with pyyaml==6.0.3 \
    pytest -q "$here/tests"
  shellcheck "$here/deploy.sh"
}

# A ZIP whose bytes depend only on controller.py, so its SHA-256 names the code.
package() {
  local out="${FM_SIM_BUILD_DIR:-$(mktemp -d)}"
  mkdir -p "$out"
  python3 -I - "$here/controller.py" "$out/controller.zip" <<'PY'
import sys
import zipfile

source, target = sys.argv[1], sys.argv[2]
info = zipfile.ZipInfo("controller.py", date_time=(1980, 1, 1, 0, 0, 0))
info.external_attr = 0o644 << 16
info.compress_type = zipfile.ZIP_DEFLATED
with open(source, "rb") as handle, zipfile.ZipFile(target, "w") as archive:
    archive.writestr(info, handle.read())
PY
  local digest
  digest="$(shasum -a 256 "$out/controller.zip" | awk '{print $1}')"
  printf '%s %s\n' "$digest" "$out/controller.zip"
}

# Appends KEY=VALUE lines from the params file to params; blanks and # comments skip.
read_params() {
  need FM_SIM_PARAMS
  [ -f "$FM_SIM_PARAMS" ] || die "params file not found: $FM_SIM_PARAMS"
  local line
  while IFS= read -r line || [ -n "$line" ]; do
    case "$line" in
      '' | '#'*) continue ;;
      [A-Z][A-Za-z0-9]*=*) params+=("$line") ;;
      *) die "bad params line: $line" ;;
    esac
  done < "$FM_SIM_PARAMS"
}

deploy() {
  local execute=0
  [ "${1:-}" = "--execute" ] && execute=1
  need FM_SIM_CODE_BUCKET FM_SIM_PARAMS
  local packaged digest zip key
  packaged="$(package)"
  digest="${packaged%% *}"
  zip="${packaged#* }"
  key="fm-sim-cloud-host/controller/$digest.zip"
  params=()
  read_params
  params+=("ControllerCodeBucket=$FM_SIM_CODE_BUCKET" "ControllerCodeKey=$key")

  if [ "$execute" = "0" ]; then
    printf 'Plan only. Rerun as "deploy.sh deploy --execute" to run:\n\n'
    printf '  aws s3api get-bucket-versioning --bucket %q   # must be Enabled\n' "$FM_SIM_CODE_BUCKET"
    printf '  aws s3 cp %q s3://%s/%s --checksum-algorithm SHA256\n' "$zip" "$FM_SIM_CODE_BUCKET" "$key"
    printf '  aws s3api head-object --bucket %q --key %q --checksum-mode ENABLED   # ChecksumSHA256 must match\n' \
      "$FM_SIM_CODE_BUCKET" "$key"
    printf '  aws cloudformation deploy --stack-name %q --template-file %q \\\n' "$stack" "$template"
    printf '    --capabilities CAPABILITY_IAM --parameter-overrides'
    printf ' %q' "${params[@]}"
    printf '\n\nStop the host first: an update that changes the group can return it to zero.\n'
    return 0
  fi
  # A versioned bucket keeps an overwritten or deleted controller recoverable.
  [ "$(aws_cli s3api get-bucket-versioning --bucket "$FM_SIM_CODE_BUCKET" --query Status --output text)" = "Enabled" ] ||
    die "enable versioning on $FM_SIM_CODE_BUCKET before deploying"
  aws_cli s3 cp "$zip" "s3://$FM_SIM_CODE_BUCKET/$key" --checksum-algorithm SHA256 --only-show-errors
  # The key names the ZIP's SHA-256; the stored object must carry that same one.
  local stored
  stored="$(aws_cli s3api head-object --bucket "$FM_SIM_CODE_BUCKET" --key "$key" \
    --checksum-mode ENABLED --query ChecksumSHA256 --output text)"
  [ "$stored" = "$(sha256_base64 "$digest")" ] ||
    die "s3://$FM_SIM_CODE_BUCKET/$key has checksum $stored, not the SHA-256 its key names"
  aws_cli cloudformation deploy --stack-name "$stack" --template-file "$template" \
    --capabilities CAPABILITY_IAM --no-fail-on-empty-changeset \
    --parameter-overrides "${params[@]}"
}

# S3 reports SHA-256 checksums in base64; the key carries hex.
sha256_base64() {
  python3 -I -c 'import base64, sys; print(base64.b64encode(bytes.fromhex(sys.argv[1])).decode())' "$1"
}

# Prints the status, and warns when spend reads zero while a host is in the group:
# the cost tag is then most likely not active in Billing, and the gate sees nothing.
status() {
  local value
  value="$(aws_cli ssm get-parameter --name "/$stack/status" --query Parameter.Value --output text)"
  printf '%s\n' "$value"
  printf '%s' "$value" | python3 -I -c '
import json, sys
status = json.load(sys.stdin)
budget = status.get("budget") or {}
if budget.get("actual_usd") == 0 and status.get("instances"):
    print("warning: a host is up but the budget reads zero spend; is the cost tag active in Billing?", file=sys.stderr)
'
}

invoke() {
  local out
  out="$(mktemp)"
  aws_cli lambda invoke --function-name "$stack-controller" \
    --cli-binary-format raw-in-base64-out --payload "{\"action\":\"$1\"}" "$out" >/dev/null
  cat "$out"; printf '\n'
  # The controller refuses in its reply, not its exit code; surface that here.
  python3 -I -c 'import json, sys; sys.exit(0 if json.load(open(sys.argv[1])).get("ok") else 1)' "$out" ||
    { rm -f "$out"; return 1; }
  rm -f "$out"
}

main() {
  local verb="${1:-}"
  [ "$#" -gt 0 ] && shift
  case "$verb" in
    validate) validate ;;
    package) package ;;
    plan) deploy ;;
    deploy) deploy "$@" ;;
    status) status ;;
    start | stop) invoke "$verb" ;;
    *) sed -n '2,/^[^#]/s/^#//p' "$0" >&2; exit 2 ;;
  esac
}

main "$@"
