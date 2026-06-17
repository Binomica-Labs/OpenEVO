#!/bin/bash
cd "$(dirname "$0")"
CYAN='\033[96m'
GREEN='\033[92m'
MAGENTA='\033[95m'
YELLOW='\033[93m'
BLUE='\033[94m'
WHITE='\033[97m'
RESET='\033[0m'

echo -e "${CYAN}============================================${RESET}"
echo -e "${MAGENTA}  OpenEvo Installer (June 4, 2026 - V1)${RESET}"
echo -e "${CYAN}============================================${RESET}"
echo ""
echo -e "${WHITE}Uses your default Python 3 (any 3.x).${RESET}"
echo ""
echo -e "${BLUE}Installing Python packages...${RESET}"
echo -e "  ${YELLOW}- nicegui==3.4.1${RESET}"
echo -e "  ${YELLOW}- pyserial${RESET}"
echo ""
pip3 install "nicegui==3.4.1" pyserial
echo ""
echo -e "${GREEN}============================================${RESET}"
echo -e "${GREEN}  Installation Complete!${RESET}"
echo -e "${GREEN}============================================${RESET}"
echo ""
echo -e "${WHITE}Next steps:${RESET}"
echo -e "  ${CYAN}1.${RESET} Upload firmware to Arduino (2026-06-04_OpenEvo_Firmware_V1.ino)"
echo -e "  ${CYAN}2.${RESET} Double-click Mac_Run_OpenEvo.command (first time: right-click -> Open)"
echo ""
echo -e "${WHITE}You can close this window.${RESET}"
