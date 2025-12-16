# Initial Setup - Physical AI Book Project

**Date:** 12/12/2025  
**Environment:** Windows 10/11, VS Code, Gemini/SpecKit CLI, PowerShell  

## 1. Project Folders
- `specs/001-physical-ai-book/contracts/validation_contracts.md`
- `history/prompts/general/initial_setup.md`

## 2. Tools & Versions
- Gemini CLI: latest
- SpecKit Plus: latest
- Node.js: v24.x
- ROS 2 Jazzy
- NVIDIA Isaac Sim 2025.2

## 3. Actions Performed
1. Created `validation_contracts.md` outlining CI/CD contracts:
   - Per-chapter validation (PR)
   - Full-book nightly integration
   - Manual release gate
2. Created folder `history/prompts/general/`
3. Created initial PHR file `initial_setup.md`
4. MCP CLI startup tested, API error `[write_file tool not found]` ignored (safe)
5. Backup of specs folder recommended

## 4. Notes
- Local files are safe; no data loss occurred
- Future PHRs can be added manually in this folder
