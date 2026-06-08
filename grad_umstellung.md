# HMI: Gelenkswinkel-Anzeige von Radiant auf Grad umstellen

Alle Slider zeigen Winkel aktuell in Radiant. Für ein industrielles HMI sind Grad intuitiver.
Folgende 4 Stellen müssen geändert werden.

---

## 1. Initialwerte von `arm_joints` ändern

```cpp
// ALT:
float arm_joints[6] = {0.0f, 0.0f, -1.5708f, -1.5708f, 0.0f, 0.0f};

// NEU:
float arm_joints[6] = {0.0f, 0.0f, -90.0f, -90.0f, 0.0f, 0.0f};
```

---

## 2. Absolute Slider — Range, Format und Konvertierung beim Senden

### Arm
```cpp
// Slider Range und Format:
ImGui::SliderFloat(arm_names[i], &arm_joints[i], -180.0f, 180.0f, "%.1f deg");

// "Joints anfahren" — vor setJointValueTarget in Radiant konvertieren:
std::vector<double> jv;
for (int i = 0; i < 6; i++) jv.push_back(arm_joints[i] * M_PI / 180.0);
```

### SCARA
```cpp
// Limits und Formate (J3 ist linear → bleibt in Meter):
float limits_lo[] = {-180.0f, -180.0f, -0.4f, -180.0f};
float limits_hi[] = { 180.0f,  180.0f,  0.0f,  180.0f};
const char* fmts[] = {"%.1f deg", "%.1f deg", "%.3f m", "%.1f deg"};

ImGui::SliderFloat(scara_names[i], &scara_joints[i], limits_lo[i], limits_hi[i], fmts[i]);

// "Joints anfahren" — J1, J2, J4 konvertieren, J3 (linear) nicht:
std::vector<double> jv;
jv.push_back(scara_joints[0] * M_PI / 180.0);
jv.push_back(scara_joints[1] * M_PI / 180.0);
jv.push_back(scara_joints[2]);                  // linear, kein convert
jv.push_back(scara_joints[3] * M_PI / 180.0);
```

---

## 3. Live-Polling Thread — Radiant → Grad beim Speichern

```cpp
// Arm (alle 6 Joints sind rotatorisch):
g_live_arm_jv[i] = (float)(jv[i] * 180.0 / M_PI);

// SCARA (J3 linear bleibt in Meter):
g_live_scara_jv[i] = (i == 2) ? (float)jv[i] : (float)(jv[i] * 180.0 / M_PI);
```

---

## 4. Live Slider — Range, Format und Konvertierung beim Senden

### Arm
```cpp
// Slider:
ImGui::SliderFloat(label.c_str(), &live_arm_edit[i], -180.0f, 180.0f, "%.1f deg");

// Beim Senden (IsItemDeactivatedAfterEdit):
jv[idx] = val * M_PI / 180.0;
```

### SCARA
```cpp
// Limits und Formate:
float lo[]          = {-180.0f, -180.0f, -0.4f, -180.0f};
float hi[]          = { 180.0f,  180.0f,  0.0f,  180.0f};
const char* fmts[]  = {"%.1f deg", "%.1f deg", "%.3f m", "%.1f deg"};

ImGui::SliderFloat(label.c_str(), &live_scara_edit[i], lo[i], hi[i], fmts[i]);

// Beim Senden (J3 linear nicht konvertieren):
jv[idx] = (idx == 2) ? val : val * M_PI / 180.0;
```

---

> **Hinweis:** Die gespeicherten Joint-Werte in der CSV bleiben in Radiant —
> die Konvertierung passiert nur in der UI-Anzeige und beim Senden an MoveIt.