# Phase 1 Setup — Windows Desktop

Follow every step in order. Do not skip ahead.

---

## Prerequisites

Install these before anything else. Each link goes to the official installer.

| Tool | Version | Download | Notes |
|------|---------|----------|-------|
| Python | 3.11 or 3.12 | https://www.python.org/downloads/ | ✅ Check **"Add Python to PATH"** during install |
| Git | Any | https://git-scm.com/download/win | Use default options |
| Flutter SDK | Latest stable | https://docs.flutter.dev/get-started/install/windows/desktop | Follow the full guide; includes Visual Studio requirement |
| Visual Studio 2022 | Community (free) | https://visualstudio.microsoft.com/ | Required by Flutter for Windows builds. Select **"Desktop development with C++"** workload |

Verify each tool is installed by opening a new Command Prompt and running:
```
python --version      → Python 3.11.x or 3.12.x
git --version         → git version 2.x
flutter --version     → Flutter 3.x
```
If any command is not found, re-install that tool and ensure it was added to PATH.

---

## Step 1 — Get the code

Open **Command Prompt** (not PowerShell — the batch scripts use cmd syntax).

```cmd
git clone https://github.com/huachong95/Projects.git
cd Projects
git checkout claude/3d-slicer-app-wjJmv
cd 3d-slicer
```

If you already cloned the repo:
```cmd
cd Projects
git checkout claude/3d-slicer-app-wjJmv
git pull
cd 3d-slicer
```

---

## Step 2 — Run the setup script

This downloads Three.js viewer assets, CuraEngine binary, and installs all dependencies.

```cmd
scripts\setup_dev.bat
```

The script will:
- Create a Python virtual environment in `backend\.venv\`
- Install all Python packages from `backend\requirements.txt`
- Download `three.min.js`, `STLLoader.js`, `OrbitControls.js` into `frontend\assets\viewer\`
- Download the CuraEngine 5.7.2 binary into `backend\slicer\cura_engine\windows\`
- Run `flutter create --platforms=windows,ios .` to generate the Windows runner
- Run `flutter pub get` to install Flutter packages

Expected duration: 3–5 minutes depending on internet speed.

If the script fails partway through, note which step failed, fix it, and re-run.

---

## Step 3 — Download CuraEngine definition files

The script downloads the CuraEngine binary, but two configuration files must be
downloaded manually (they are too large to bundle and are version-specific):

1. Go to:
   https://github.com/Ultimaker/Cura/tree/5.7.2/resources/definitions

2. Download both files (click the file → Raw → Save As):
   - `fdmprinter.def.json`
   - `fdmextruder.def.json`

3. Place both files in:
   ```
   3d-slicer\backend\slicer\cura_profiles\definitions\
   ```

> **Note for Phase 1:** This step is only needed to confirm `cura_engine_ready: true`
> in the health check. Slicing is tested in Phase 2, so this can be done after
> the rest of Phase 1 passes.

---

## Step 4 — Start the backend

Open a **new Command Prompt window** and run:

```cmd
cd Projects\3d-slicer\backend
.venv\Scripts\activate
python main.py
```

You should see:
```
INFO:     Started server process [...]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

Leave this window open. The backend must stay running while you use the app.

**Verify it's working** — open a browser and go to:
```
http://localhost:8000/health
```
Expected response:
```json
{"status":"ok","cura_engine_ready":false,"ai_model_ready":false,"printer_connected":false}
```
`false` values are expected at this point — they become `true` in later phases.

---

## Step 5 — Run the Flutter app

Open a **second Command Prompt window** and run:

```cmd
cd Projects\3d-slicer\frontend
flutter run -d windows
```

First run compiles everything — takes 2–4 minutes. Subsequent runs are faster.

You should see a window open with the 3D Slicer home screen.

---

## Step 6 — Verify Phase 1

Work through the Phase 1 checklist in `TESTING.md`:

1. Home screen shows a **green dot** (backend reachable)
2. Tap **Import Model** → pick any `.stl` file → mesh info card appears
3. Tap **Connect Printer** → screen opens with IP/API key fields

All three items passing = **Phase 1 complete**.

---

## Troubleshooting

**`python` not found**
Re-run the Python installer and check "Add Python to PATH". Then open a fresh Command Prompt.

**`flutter` not found**
Follow https://docs.flutter.dev/get-started/install/windows/desktop fully, including
adding Flutter to PATH and running `flutter doctor`.

**`flutter run` fails with "Unable to find suitable development target"**
Run `flutter doctor` and fix any issues it reports, especially Visual Studio.

**Backend starts but browser shows "connection refused"**
The backend defaults to port 8000. Check nothing else is using that port:
```cmd
netstat -ano | findstr :8000
```

**`flutter pub get` fails on a package**
Make sure you are on Flutter stable channel: `flutter channel stable && flutter upgrade`

**setup_dev.bat fails at CuraEngine download**
CuraEngine can be downloaded manually:
https://github.com/Ultimaker/CuraEngine/releases/tag/5.7.2
Place the `.exe` at: `backend\slicer\cura_engine\windows\CuraEngine.exe`

---

## What each terminal window should show when Phase 1 is running

**Terminal 1 (backend):**
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

**Terminal 2 (Flutter app):**
```
flutter: Observatory listening on http://127.0.0.1:...
```
Plus the slicer app window open on screen.
