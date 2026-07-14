#!/usr/bin/env bash
set -euo pipefail

# Usage : ./update_hw.sh [version]
# Exemple : ./update_hw.sh v1.0.1   (ou sans argument -> "latest")

VERSION="${1:-latest}"
REPO_HW="Les-opossums/Zynq_HW_Eurobot_2025"
DEST_DIR="./opossum_hw"

# --- Verification des dependances ---
if ! command -v curl >/dev/null 2>&1; then
    echo "ERREUR : 'curl' n'est pas installe ou pas dans le PATH." >&2
    exit 1
fi

PYTHON_CMD=""
for cmd in python3 python; do
    if command -v "$cmd" >/dev/null 2>&1; then
        PYTHON_CMD="$cmd"
        break
    fi
done
if [[ -z "$PYTHON_CMD" ]]; then
    echo "ERREUR : python n'est pas installe ou pas dans le PATH (necessaire pour parser le JSON)." >&2
    exit 1
fi

echo "Initialisation de la mise a jour materielle (Version cible: $VERSION)..."

# 1. Preparation du repertoire
mkdir -p "$DEST_DIR"
rm -f "$DEST_DIR"/*.xsa

# 2. Determination de l'URL de l'API en fonction du parametre
# NB : /releases/latest ignore les pre-releases (ex: tags -alpha/-beta),
# on utilise donc la liste complete et on prend la premiere entree.
if [[ "$VERSION" == "latest" ]]; then
    API_URL="https://api.github.com/repos/${REPO_HW}/releases"
else
    API_URL="https://api.github.com/repos/${REPO_HW}/releases/tags/${VERSION}"
fi

# 3. Recuperation des metadonnees de la release
HTTP_CODE="$(curl -sSL -o /tmp/release_response.json -w "%{http_code}" "$API_URL")"

if [[ "$HTTP_CODE" != "200" ]]; then
    echo "ERREUR : Impossible de trouver la release '$VERSION' (HTTP $HTTP_CODE)." >&2
    echo "Reponse de l'API GitHub :" >&2
    cat /tmp/release_response.json >&2
    rm -f /tmp/release_response.json
    exit 1
fi

# 4. Selection de l'asset .xsa (premier trouve) via python
PARSED="$("$PYTHON_CMD" -c '
import json, sys
raw = json.load(open(sys.argv[1], encoding="utf-8"))
# /releases renvoie une liste (on prend la plus recente) ; /releases/tags/X renvoie un objet
if isinstance(raw, list):
    if not raw:
        print("")
        print("")
        print("")
        print("")
        sys.exit(0)
    data = raw[0]
else:
    data = raw
assets = [a for a in data.get("assets", []) if a["name"].endswith(".xsa")]
name = assets[0]["name"] if assets else ""
url = assets[0]["browser_download_url"] if assets else ""
tag = data.get("tag_name", "")
prerelease = "1" if data.get("prerelease") else "0"
print(name)
print(url)
print(tag)
print(prerelease)
' /tmp/release_response.json)"
rm -f /tmp/release_response.json

ASSET_NAME="$(echo "$PARSED" | sed -n '1p')"
ASSET_URL="$(echo "$PARSED" | sed -n '2p')"
TAG_NAME="$(echo "$PARSED" | sed -n '3p')"
IS_PRERELEASE="$(echo "$PARSED" | sed -n '4p')"

if [[ -z "$ASSET_URL" ]]; then
    echo "ERREUR : Aucun fichier .xsa trouve pour cette release." >&2
    exit 1
fi

if [[ "$IS_PRERELEASE" == "1" ]]; then
    echo "ATTENTION : la release '${TAG_NAME}' est une PRE-RELEASE (non stable)." >&2
fi

# 5. Telechargement
FILE_PATH="${DEST_DIR}/${ASSET_NAME}"
echo "Telechargement de '${ASSET_NAME}' (Release: ${TAG_NAME})..."
curl -fsSL -o "$FILE_PATH" "$ASSET_URL"

echo "SUCCES : Configuration '${ASSET_NAME}' prete dans ${DEST_DIR}."