#!/usr/bin/env python3
"""
ZB-BSB — Generateur d'image Zigbee OTA (.ota) et de manifeste serveur (index.json).

Fonctionne a la fois :
  1. Automatiquement a chaque compilation PlatformIO (`pio run`) via `extra_scripts`.
  2. En ligne de commande autonome :
     python scripts/build_ota.py --base-url https://ota.mon-domaine.fr/zb-bsb --version 0x01020000
"""

import argparse
import hashlib
import json
import os
import struct
import sys
from pathlib import Path

# Constantes Zigbee OTA standard (ZCL Spec 11.4.2)
OTA_UPGRADE_FILE_ID      = 0x0BEEF11E
OTA_HEADER_VERSION       = 0x0100
OTA_HEADER_LENGTH        = 56
OTA_HEADER_FIELD_CONTROL = 0x0000
ZIGBEE_STACK_VERSION_PRO = 0x0002
OTA_SUB_ELEMENT_TAG_IMG  = 0x0000
OTA_ELEMENT_HEADER_LEN   = 6

DEFAULT_MANUFACTURER_CODE = 0x131B  # 4891
DEFAULT_IMAGE_TYPE        = 0x0001
DEFAULT_FILE_VERSION      = 0x01010000
DEFAULT_HW_VERSION        = 0x0001
DEFAULT_HEADER_STRING     = "Atlantic-Siemens ZB-BSB OTA"
DEFAULT_BASE_URL          = "https://ota.example.com/zb-bsb"


def parse_int(val, default):
    if val is None:
        return default
    if isinstance(val, int):
        return val
    s = str(val).strip()
    if not s:
        return default
    return int(s, 0)


def build_zigbee_ota_image(
    firmware_bin_path: Path,
    output_ota_path: Path,
    file_version: int,
    manufacturer_code: int = DEFAULT_MANUFACTURER_CODE,
    image_type: int = DEFAULT_IMAGE_TYPE,
    header_string: str = DEFAULT_HEADER_STRING,
) -> bytes:
    """Empaquette un binaire ESP32 (.bin) dans un conteneur Zigbee OTA standard (.ota)."""
    fw_data = firmware_bin_path.read_bytes()
    fw_size = len(fw_data)

    # Total image size = 56 (OTA Header) + 6 (Sub-element Header) + len(firmware.bin)
    total_image_size = OTA_HEADER_LENGTH + OTA_ELEMENT_HEADER_LEN + fw_size

    header_str_bytes = header_string.encode("ascii", errors="ignore")[:32]
    header_str_padded = header_str_bytes.ljust(32, b"\x00")

    # Zigbee OTA Header (56 octets, Little-Endian)
    # <I H H H H H I H 32s I
    ota_header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_UPGRADE_FILE_ID,
        OTA_HEADER_VERSION,
        OTA_HEADER_LENGTH,
        OTA_HEADER_FIELD_CONTROL,
        manufacturer_code & 0xFFFF,
        image_type & 0xFFFF,
        file_version & 0xFFFFFFFF,
        ZIGBEE_STACK_VERSION_PRO,
        header_str_padded,
        total_image_size,
    )
    assert len(ota_header) == OTA_HEADER_LENGTH

    # Sub-element Header (6 octets : Tag ID = 0x0000 Upgrade Image + Longueur = fw_size)
    sub_element_header = struct.pack("<HI", OTA_SUB_ELEMENT_TAG_IMG, fw_size)
    assert len(sub_element_header) == OTA_ELEMENT_HEADER_LEN

    ota_blob = ota_header + sub_element_header + fw_data
    output_ota_path.parent.mkdir(parents=True, exist_ok=True)
    output_ota_path.write_bytes(ota_blob)
    return ota_blob


def generate_ota_artifacts(
    firmware_bin_path: Path,
    ota_dir: Path,
    file_version: int,
    manufacturer_code: int,
    image_type: int,
    base_url: str,
):
    if not firmware_bin_path.is_file():
        print(f"[ZB-BSB OTA] Fichier binaire introuvable : {firmware_bin_path}", file=sys.stderr)
        return False

    ota_dir.mkdir(parents=True, exist_ok=True)
    ver_hex = f"{file_version:08X}"
    versioned_filename = f"zb_bsb_v{ver_hex}.ota"
    latest_filename = "zb_bsb.ota"

    latest_ota_path = ota_dir / latest_filename
    versioned_ota_path = ota_dir / versioned_filename

    ota_blob = build_zigbee_ota_image(
        firmware_bin_path=firmware_bin_path,
        output_ota_path=latest_ota_path,
        file_version=file_version,
        manufacturer_code=manufacturer_code,
        image_type=image_type,
    )
    versioned_ota_path.write_bytes(ota_blob)

    sha512_hex = hashlib.sha512(ota_blob).hexdigest()
    sha256_hex = hashlib.sha256(ota_blob).hexdigest()
    clean_base_url = base_url.rstrip("/")
    file_url = f"{clean_base_url}/{versioned_filename}"

    # Format d'index JSON compatible ZHA (zigpy z2m extra_providers) & Zigbee2MQTT
    index_data = [
        {
            "fileName": versioned_filename,
            "fileVersion": file_version,
            "fileSize": len(ota_blob),
            "url": file_url,
            "imageType": image_type,
            "manufacturerCode": manufacturer_code,
            "sha512": sha512_hex,
            "sha256": sha256_hex,
            "otaHeaderString": DEFAULT_HEADER_STRING,
            "manufacturerName": ["Atlantic-Siemens"],
            "modelId": "ZB-BSB",
        }
    ]

    index_path = ota_dir / "index.json"
    index_path.write_text(json.dumps(index_data, indent=2) + "\n", encoding="utf-8")

    print("=" * 72)
    print(f"[ZB-BSB OTA] Image Zigbee OTA generee avec succes :")
    print(f"  - Fichier OTA (latest)    : {latest_ota_path}")
    print(f"  - Fichier OTA (versionne) : {versioned_ota_path}")
    print(f"  - Manifeste Serveur HTTP  : {index_path}")
    print(f"  - Version Firmware (hex)  : 0x{ver_hex} ({file_version})")
    print(f"  - Manufacturer / Type     : 0x{manufacturer_code:04X} ({manufacturer_code}) / 0x{image_type:04X}")
    print(f"  - Taille Image OTA        : {len(ota_blob)} octets")
    print(f"  - URL cible dans index    : {file_url}")
    print("=" * 72)
    return True


# ============================================================================
# Mode 1 : Hook automatique PlatformIO (`extra_scripts = scripts/build_ota.py`)
# ============================================================================
try:
    Import("env")  # type: ignore[name-defined]
    IN_PLATFORMIO = True
except Exception:
    IN_PLATFORMIO = False

if IN_PLATFORMIO:
    raw_ver = os.environ.get("ZB_BSB_OTA_VERSION") or env.GetProjectOption("custom_ota_version", None)  # type: ignore[name-defined]
    raw_manuf = os.environ.get("ZB_BSB_OTA_MANUFACTURER") or env.GetProjectOption("custom_ota_manufacturer", None)  # type: ignore[name-defined]
    raw_type = os.environ.get("ZB_BSB_OTA_IMAGE_TYPE") or env.GetProjectOption("custom_ota_image_type", None)  # type: ignore[name-defined]
    raw_hw = os.environ.get("ZB_BSB_OTA_HW_VERSION") or env.GetProjectOption("custom_ota_hw_version", None)  # type: ignore[name-defined]
    raw_url = os.environ.get("ZB_BSB_OTA_BASE_URL") or env.GetProjectOption("custom_ota_base_url", DEFAULT_BASE_URL)  # type: ignore[name-defined]

    ota_version = parse_int(raw_ver, DEFAULT_FILE_VERSION)
    ota_manuf = parse_int(raw_manuf, DEFAULT_MANUFACTURER_CODE)
    ota_img_type = parse_int(raw_type, DEFAULT_IMAGE_TYPE)
    ota_hw_ver = parse_int(raw_hw, DEFAULT_HW_VERSION)
    ota_base_url = str(raw_url).strip()

    # Injecte les macros C++ avant la compilation pour garantir la coherence binaire <-> en-tete OTA
    env.Append(  # type: ignore[name-defined]
        CPPDEFINES=[
            ("ZB_BSB_OTA_FILE_VERSION", f"0x{ota_version:08X}UL"),
            ("ZB_BSB_OTA_MANUFACTURER_CODE", f"0x{ota_manuf:04X}"),
            ("ZB_BSB_OTA_IMAGE_TYPE", f"0x{ota_img_type:04X}"),
            ("ZB_BSB_OTA_HW_VERSION", f"0x{ota_hw_ver:04X}"),
        ]
    )

    def _pio_post_build_ota(source, target, env):
        project_dir = Path(env.subst("$PROJECT_DIR"))
        firmware_bin = Path(env.subst("$BUILD_DIR/${PROGNAME}.bin"))
        ota_dir = project_dir / "ota"
        generate_ota_artifacts(
            firmware_bin_path=firmware_bin,
            ota_dir=ota_dir,
            file_version=ota_version,
            manufacturer_code=ota_manuf,
            image_type=ota_img_type,
            base_url=ota_base_url,
        )

    env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", _pio_post_build_ota)  # type: ignore[name-defined]


# ============================================================================
# Mode 2 : Execution autonome en ligne de commande
# ============================================================================
def main():
    repo_root = Path(__file__).resolve().parent.parent
    default_bin = repo_root / ".pio" / "build" / "seeed_xiao_esp32c6" / "firmware.bin"
    default_ota_dir = repo_root / "ota"

    parser = argparse.ArgumentParser(
        description="Empaquette firmware.bin en image Zigbee OTA (.ota) et genere index.json pour un serveur HTTP/HTTPS personnalise."
    )
    parser.add_argument(
        "--firmware",
        type=Path,
        default=default_bin,
        help=f"Chemin vers firmware.bin (defaut: {default_bin})",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=default_ota_dir,
        help=f"Dossier de sortie OTA (defaut: {default_ota_dir})",
    )
    parser.add_argument(
        "--version",
        type=str,
        default=hex(DEFAULT_FILE_VERSION),
        help=f"Version du fichier OTA en hex ou decimal (defaut: 0x{DEFAULT_FILE_VERSION:08X})",
    )
    parser.add_argument(
        "--manufacturer",
        type=str,
        default=hex(DEFAULT_MANUFACTURER_CODE),
        help=f"Code fabricant Zigbee (defaut: 0x{DEFAULT_MANUFACTURER_CODE:04X} = {DEFAULT_MANUFACTURER_CODE})",
    )
    parser.add_argument(
        "--image-type",
        type=str,
        default=hex(DEFAULT_IMAGE_TYPE),
        help=f"Type d'image OTA (defaut: 0x{DEFAULT_IMAGE_TYPE:04X})",
    )
    parser.add_argument(
        "--base-url",
        type=str,
        default=DEFAULT_BASE_URL,
        help=f"URL de base du serveur HTTP/HTTPS hebergeant les fichiers OTA (defaut: {DEFAULT_BASE_URL})",
    )

    args = parser.parse_args()
    ok = generate_ota_artifacts(
        firmware_bin_path=args.firmware,
        ota_dir=args.out_dir,
        file_version=parse_int(args.version, DEFAULT_FILE_VERSION),
        manufacturer_code=parse_int(args.manufacturer, DEFAULT_MANUFACTURER_CODE),
        image_type=parse_int(args.image_type, DEFAULT_IMAGE_TYPE),
        base_url=args.base_url,
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__" and not IN_PLATFORMIO:
    main()
