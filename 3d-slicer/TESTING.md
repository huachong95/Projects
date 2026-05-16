# 3D Slicer — Phase Testing Checklist

Each phase must be fully verified before starting the next.
A ✅ means the test passes. A ❌ means a bug to fix before proceeding.

---

## Phase 1 — Core Infrastructure
**Goal:** Backend starts cleanly. Flutter app launches. File upload works.
No printer, no slicing, no camera needed.

### Setup
```
cd 3d-slicer
scripts/setup_dev.bat          # Windows (downloads Three.js, CuraEngine, Flutter deps)
cd backend && .venv\Scripts\activate && python main.py
# In a second terminal:
cd frontend && flutter run -d windows
```

### Backend tests (automated)
```
cd backend && python -m pytest tests/ -v
```
Expected: **9 passed**

### Backend health check (manual — open in browser or curl)
```
curl http://localhost:8000/health
```
Expected response:
```json
{
  "status": "ok",
  "cura_engine_ready": false,   ← false until CuraEngine binary downloaded
  "ai_model_ready": false,      ← false until model downloaded
  "printer_connected": false
}
```
- [ ] `/health` returns HTTP 200
- [ ] `status` is `"ok"`

### File upload (manual — use the Flutter app)
1. Launch Flutter app → Home screen appears with backend status dot
- [ ] Green dot visible (backend reachable)
- [ ] No crash on startup

2. Tap **Import Model** → tap **Browse Files** → select any `.stl` file
- [ ] Upload completes (no spinner stuck)
- [ ] Mesh info card shows filename, triangle count, dimensions
- [ ] "Slice Settings" button appears

3. Tap **Connect Printer** from Home
- [ ] Screen opens with "No printer connected" card
- [ ] "Discover on network" button is present
- [ ] IP and API key fields are present

### Phase 1 complete when:
All items above are checked. No printer connection required yet.

---

## Phase 2 — Slicing Pipeline
**Goal:** Upload a real STL, slice it with CuraEngine, view layers, download G-code.
No printer needed.

### Setup (in addition to Phase 1)
1. Download CuraEngine definition files from:
   `https://github.com/Ultimaker/Cura/tree/5.7.2/resources/definitions`
   - Place `fdmprinter.def.json` → `backend/slicer/cura_profiles/definitions/`
   - Place `fdmextruder.def.json` → `backend/slicer/cura_profiles/definitions/`
2. Verify: `curl http://localhost:8000/health` → `cura_engine_ready: true`

### Test STL
Download the Prusa Mk4 test file or use any small STL (e.g. a 20mm calibration cube).

### Slicing flow (manual — Flutter app)
1. Import a small STL (≤50 triangles for fast slicing)
- [ ] Mesh info card shows correct dimensions

2. Tap **Slice Settings**
   - Profile selector shows: `pla_fine`, `pla_standard`, `petg_standard`, `prusa_mk4`
   - [ ] All four profiles visible in dropdown

3. Set layer height 0.2mm, infill 15%, no supports → tap **Slice**
   - [ ] Progress bar fills (0 → 100%)
   - [ ] No error message appears

4. After slicing → viewer opens
   - [ ] 3D model renders (orange mesh on dark background)
   - [ ] "Layers" toggle button appears in app bar

5. Tap **Layers** → drag layer slider
   - [ ] Layer lines visible (blue = perimeter, grey = infill)
   - [ ] Slider updates the displayed layer

6. Back-end: verify G-code via curl:
   ```
   curl http://localhost:8000/api/slice/<slice_job_id>/metadata
   ```
   - [ ] `layer_count` > 0
   - [ ] `estimated_time_seconds` > 0
   - [ ] `filament_used_mm` > 0

7. Download G-code:
   ```
   curl http://localhost:8000/api/slice/<slice_job_id>/gcode -o test.gcode
   ```
   - [ ] File downloads successfully
   - [ ] Opening `test.gcode` in a text editor shows `;LAYER:0`, `;LAYER:1`, etc.
   - [ ] File contains `; HOST_ACTION:TIMELAPSE_CAPTURE` lines (timelapse hooks)

### Phase 2 complete when:
All items above are checked. G-code file is valid and openable in PrusaSlicer for comparison.

---

## Phase 3 — Printer Connectivity
**Goal:** Connect to Prusa Mk4 via PrusaLink. Upload and start a real print. Monitor
temperatures and print progress in real-time.

### Hardware needed
- Prusa Mk4 powered on and connected to Wi-Fi
- Prusa Mk4 IP address (Settings → Network → IP Address on the printer screen)
- PrusaLink API key (Settings → Network → PrusaLink on the printer screen)

### Connection (Flutter app)
1. Home → **Connect Printer**
2. Tap **Discover on network**
   - [ ] Printer appears in discovered list within 5 seconds, OR
   - [ ] Enter IP manually if discovery fails

3. Enter IP and API key → tap **Connect**
   - [ ] "Prusa Mk4 connected" card turns green
   - [ ] No error message

4. Verify via backend:
   ```
   curl http://localhost:8000/api/printer/status
   ```
   - [ ] Response includes `temp_hotend`, `temp_bed`, `state`
   - [ ] `state` is `"IDLE"` when printer is idle

### Print flow (Flutter app)
1. Upload + slice a small model (use 20mm calibration cube, ~5 min print time)

2. From Slice Settings screen → tap **Send to Printer** (or use curl):
   ```
   curl -X POST http://localhost:8000/api/printer/print \
     -H "Content-Type: application/json" \
     -d '{"slice_job_id":"<id>","filename":"test.gcode"}'
   ```
   - [ ] Print starts on the Mk4 (display shows print name)

3. Home → **Print Monitor**
   - [ ] Progress bar increments over time
   - [ ] Nozzle temperature shows ~215°C
   - [ ] Bed temperature shows ~60°C
   - [ ] Layer counter increments

4. Tap **Pause** → verify printer pauses
   - [ ] State changes to `"PAUSED"` in monitor
   - [ ] Printer physically pauses

5. Tap **Resume** → verify printer resumes
   - [ ] State changes back to `"PRINTING"`

6. Tap **Cancel** → confirm dialog → cancel print
   - [ ] Print stops on Mk4
   - [ ] State changes to `"IDLE"`

### Phase 3 complete when:
Full print cycle (start → pause → resume → cancel) works from the Flutter app.

---

## Phase 4 — Camera Feed & Timelapse
**Goal:** Live Prusa Camera feed visible in the app. Timelapse frames captured
automatically during a print. FFmpeg compiles them into a watchable MP4.

### Hardware needed
- Prusa Camera installed and connected to the Mk4 via USB
- Printer connected (Phase 3 complete)

### Camera feed (Flutter app)
1. Connect to printer (Phase 3 steps 1–3)

2. Home → **Print Monitor**
   - [ ] Camera feed area shows a live image (not "No camera feed")
   - [ ] Image updates visibly (~5 fps)
   - [ ] Camera shows the printer bed

3. Test camera API directly:
   ```
   curl http://localhost:8000/api/monitoring/camera/snapshot -o snapshot.jpg
   ```
   - [ ] `snapshot.jpg` opens as a valid JPEG
   - [ ] Image shows the print bed

### Timelapse (requires a real print)
1. Set up and start a print (Phase 3 steps)

2. During print, check frame capture via backend:
   ```
   curl http://localhost:8000/api/timelapse
   ```
   - [ ] Response shows a job with increasing `frame_count`
   - [ ] `frame_count` increments once per layer change

3. After print completes, compile timelapse:
   ```
   curl -X POST http://localhost:8000/api/timelapse/<job_id>/compile
   ```
   - [ ] Response: `{"status": "compiled", "path": "..."}`

4. Home → **Timelapses**
   - [ ] Completed job card appears with video icon
   - [ ] Tap → video plays in the app
   - [ ] Video shows the model growing layer by layer

### Phase 4 complete when:
A full timelapse video of a real print is viewable in the app.

---

## Phase 5 — AI Failure Detection
**Goal:** App detects spaghetti/detachment in the camera feed and shows an alert.

### Setup
```
scripts/download_ai_model.bat   # downloads failure_detector.onnx (~25 MB)
```
Verify: `curl http://localhost:8000/health` → `ai_model_ready: true`

### Offline test (no live print needed)
```
curl -X POST http://localhost:8000/api/monitoring/ai/enable
```

1. Point the camera at a sheet of spaghetti or tangled string on the print bed
   (simulates a failure)

2. Wait ~10 seconds

3. Check AI status:
   ```
   curl http://localhost:8000/api/monitoring/ai/status
   ```
   - [ ] `failure_probability` > 0.5 for the spaghetti image

4. Point camera back at clean print bed
   - [ ] `failure_probability` drops below 0.3

### Live print test
1. Start a print and enable AI monitoring:
   ```
   curl -X POST http://localhost:8000/api/monitoring/ai/enable
   ```
2. Home → **Print Monitor**
   - [ ] No alert during normal printing
   - [ ] AI detection runs silently in background (no performance impact on UI)

3. (Optional) Induce a failure (pause print, pull filament out, resume)
   - [ ] Orange alert banner slides in within ~15 seconds
   - [ ] Banner shows failure probability percentage
   - [ ] "Pause Print" button in banner works

### Phase 5 complete when:
AI correctly distinguishes a failure image from a clean bed with > 70% confidence.

---

## Phase 6 — Cloud Sync & iPhone
**Goal:** Sign in on both Windows and iPhone. Profiles sync between devices.
iPhone can control and monitor a print over Wi-Fi.

### Setup
1. Create a Firebase project at console.firebase.google.com
2. Enable: Authentication (Google Sign-In), Firestore, Storage
3. Run in `frontend/`: `flutterfire configure`
   - This generates `lib/core/firebase_options.dart`
4. Build the iOS app on a Mac: `flutter build ipa`

### Cloud sync (Windows app)
1. Home → **Cloud Sync** (new screen, built in this phase)
2. Tap **Sign in with Google**
   - [ ] Google sign-in dialog opens
   - [ ] Signed-in successfully

3. Tap **Push profiles to cloud**
   - [ ] Firestore shows the profiles in Firebase console

### iPhone test
1. On iPhone: open app → Cloud Sync → sign in with the same Google account
   - [ ] Profiles downloaded from Firestore match what was pushed from Windows

2. On iPhone: set backend URL to `http://<windows-ip>:8000`
   - [ ] Home screen shows green dot (backend reachable over Wi-Fi)

3. On iPhone: Import Model → upload an STL
   - [ ] Upload succeeds

4. On iPhone: Slice Settings → slice the model
   - [ ] Slicing progress visible on iPhone
   - [ ] Layer preview works on iPhone

5. On iPhone: Print Monitor during an active print
   - [ ] Camera feed visible on iPhone
   - [ ] Pause/Resume works from iPhone

### Phase 6 complete when:
Full slicing + monitoring workflow works on iPhone over home Wi-Fi.

---

## Phase 7 — Polish & Edge Cases
**Goal:** App is stable under real-world conditions.

- [ ] Backend survives printer disconnecting mid-print (no crash, reconnect attempt)
- [ ] Upload of a malformed STL shows a clear error message (not a crash)
- [ ] Slicing a very large STL (>1M triangles) completes without timeout
- [ ] Camera feed loss mid-print shows "no feed" placeholder (not a frozen frame)
- [ ] App works when Windows and iPhone are on different subnets (mDNS won't work — manual IP entry required and documented)
- [ ] Timelapse with 500+ layers compiles within 60 seconds
- [ ] AI monitor disabled when printer is idle (no wasted CPU)
- [ ] All temperatures displayed with correct units (°C)
- [ ] G-code download works for files > 10 MB

---

## Quick reference — API smoke tests

Run these after every code change to catch regressions:
```bash
# Start backend first
curl http://localhost:8000/health                                      # 200 ok
curl -X POST http://localhost:8000/api/mesh/upload -F file=@cube.stl  # 200 job_id
curl http://localhost:8000/api/slice/profiles                          # 200 list
curl http://localhost:8000/api/printer/discover                        # 200 devices[]
curl http://localhost:8000/api/timelapse                               # 200 jobs[]
```
