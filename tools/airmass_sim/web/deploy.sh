#!/bin/sh
# Publish the browser replay to https://airmass-replay.<subdomain>.workers.dev (Cloudflare Workers static assets).
# Needs wrangler logged in (or CLOUDFLARE_API_TOKEN with Workers Scripts: Edit) and airmass.js built (build.sh).
#   tools/airmass_sim/web/deploy.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SCRIPTS="gliders.js convert.js replay.js airmass.js" # loaded by index.html
FILES="index.html $SCRIPTS"
for f in $FILES; do
    [ -f "$HERE/$f" ] || { echo "missing $HERE/$f (run build.sh for airmass.js)" >&2; exit 1; }
done
# only the page files go public, not the sources and tools next to them
DIST=$(mktemp -d)
trap 'rm -rf "$DIST"' EXIT
for f in $FILES; do cp "$HERE/$f" "$DIST/"; done
# the script links get a content hash, so browsers do not mix a new page with cached scripts
H=$(cd "$HERE" && cat $SCRIPTS | shasum | cut -c1-10)
for f in $SCRIPTS; do
    sed -i.bak "s|src=\"$f\"|src=\"$f?v=$H\"|" "$DIST/index.html"
    grep -q "src=\"$f?v=$H\"" "$DIST/index.html" || { echo "index.html does not load $f" >&2; exit 1; }
done
rm -f "$DIST/index.html.bak"
npx --yes wrangler@4 deploy --assets "$DIST" --name airmass-replay --compatibility-date 2026-11-04
