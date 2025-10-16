#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Finger Tracking -> Serial 5 Servos
OPEN -> 0°, CLOSE -> 180°  (untuk semua jari lewat kalibrasi)
Format TX: F,<Thumb>,<Index>,<Middle>,<Ring>,<Pinky>\n
Keys:
  O / [ : Simpan OPEN (tangan terbuka) untuk semua jari
  C / ] : Simpan CLOSE (tangan tertutup) untuk semua jari
  R     : Reset kalibrasi
  P     : Pause
  Q     : Quit
"""

import sys, time, math
from typing import Dict, Tuple, Optional

import cv2
import numpy as np

# ====== Serial config ======
SERIAL_PORT = "COM5"      # Windows: "COM3" ; Linux/Mac: "/dev/ttyACM0" / "/dev/ttyUSB0"
SERIAL_BAUD = 115200
SERIAL_SEND_EVERY_MS = 33  # ~30 Hz

try:
    import serial
except ImportError:
    serial = None
    print("[WARN] 'pyserial' belum terpasang. Jalankan: pip install pyserial")

# ====== MediaPipe ======
try:
    import mediapipe as mp
except ImportError:
    print("pip install mediapipe==0.10.14 opencv-python numpy")
    sys.exit(1)

CAM_INDEX = 0
FRAME_W   = 1280
FRAME_H   = 720

SMOOTH_ALPHA = 0.30
SWAP_HANDEDNESS_FOR_VIEWER = True
SHOW_DEG_IN_PANEL = True  # tampilkan sudut servo (0..180) di panel

mp_hands   = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles  = mp.solutions.drawing_styles

FINGERS = ['Thumb','Index','Middle','Ring','Pinky']
TIP_IDS = [4, 8, 12, 16, 20]

# ---------- Utils ----------
def lm_to_pixels(lm, w, h): return int(lm.x*w), int(lm.y*h)
def np3(lm): return np.array([lm.x, lm.y, lm.z], dtype=np.float32)
def _unit(v): n=np.linalg.norm(v)+1e-8; return v/n
def angle_between(a,b):
    an=_unit(a); bn=_unit(b)
    c=float(np.clip(np.dot(an,bn),-1.0,1.0))
    return float(np.degrees(np.arccos(c)))
def angle_degrees_3d(a,b,c):
    v1=a-b; v2=c-b; return angle_between(v1,v2)
def _swap_lr(lbl): 
    return 'Right' if lbl.lower().startswith('left') else ('Left' if lbl.lower().startswith('right') else lbl)

# Sudut 3D per jari (Thumb: CMC-MCP-IP; lainnya: MCP-PIP-DIP)
def compute_finger_angles_3d(lms)->Dict[str,float]:
    pts=[np3(lm) for lm in lms]
    return {
        'Thumb' : angle_degrees_3d(pts[1],  pts[2],  pts[3]),
        'Index' : angle_degrees_3d(pts[5],  pts[6],  pts[7]),
        'Middle': angle_degrees_3d(pts[9],  pts[10], pts[11]),
        'Ring'  : angle_degrees_3d(pts[13], pts[14], pts[15]),
        'Pinky' : angle_degrees_3d(pts[17], pts[18], pts[19]),
    }

class EmaSmoother:
    def __init__(self, alpha=0.3):
        self.a=float(alpha); self.state={}
    def update(self, hand_label, key, val):
        k=(hand_label,key)
        if k not in self.state: self.state[k]=val; return val
        p=self.state[k]; s=self.a*val+(1-self.a)*p; self.state[k]=s; return s

# ---------- Kalibrasi OPEN/CLOSE untuk semua jari ----------
class MultiFingerBounds:
    """
    Simpan OPEN/CLOSE per jari:
      db[label_hand][finger] = {'open':deg, 'close':deg}
    Mapping servo: OPEN -> 0°, CLOSE -> 180°
    """
    def __init__(self):
        self.db={}  # dict: hand_label -> finger -> dict

    def _ensure(self, label):
        if label not in self.db: self.db[label]={}
        for f in FINGERS:
            self.db[label].setdefault(f, {})

    def set_open_from_angles(self, label, angles:Dict[str,float]):
        self._ensure(label)
        for f in FINGERS: self.db[label][f]['open']=float(angles[f])

    def set_close_from_angles(self, label, angles:Dict[str,float]):
        self._ensure(label)
        for f in FINGERS: self.db[label][f]['close']=float(angles[f])

    def has_pair(self, label)->bool:
        if label not in self.db: return False
        for f in FINGERS:
            d=self.db[label][f]
            if 'open' not in d or 'close' not in d: return False
            if abs(d['close']-d['open'])<=1e-6: return False
        return True

    def _ordered(self, v_open, v_close):
        # Tidak masalah mana lebih besar; normalisasi akan mengembalikan 0 di open, 1 di close
        return v_open, v_close

    def norm01(self, label, finger, angle)->float:
        """0 di OPEN, 1 di CLOSE. Jika belum ada kalibrasi, fallback 0..180 (dibalik supaya OPEN≈0)."""
        if not self.has_pair(label):
            # fallback: OPEN kira-kira 180, CLOSE kira-kira kecil -> 0..180 lalu dibalik
            v01 = np.clip((angle - 0.0)/(180.0-0.0+1e-8), 0.0, 1.0)  # 0°->0, 180°->1
            return 1.0 - v01  # OPEN(≈180)->0, CLOSE(≈kecil)->~1
        d=self.db[label][finger]
        open_v, close_v = self._ordered(d['open'], d['close'])
        # Normalisasi 0..1 dari open->close (apapun urutannya)
        v01 = (angle - open_v) / ((close_v - open_v) + 1e-8)
        return float(np.clip(v01, 0.0, 1.0))

    def servo_deg(self, label, finger, angle)->float:
        """Kembalikan derajat servo 0..180 (OPEN->0, CLOSE->180)."""
        return 180.0 * self.norm01(label, finger, angle)

    def debug_open_close(self, label):
        if not self.has_pair(label): return None
        out={}
        for f in FINGERS:
            d=self.db[label][f]; out[f]=(d['open'], d['close'])
        return out

def draw_panel(img, deg_map:Dict[str,float], origin=(10,60), bar_w=260, bar_h=16, gap=8, title="Servos (0=open,180=close)"):
    x0,y0=origin
    cv2.putText(img, title, (x0,y0-12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200),1,cv2.LINE_AA)
    for i,f in enumerate(FINGERS):
        val = float(np.clip(deg_map[f]/180.0, 0.0, 1.0))  # 0..1
        y = y0 + i*(bar_h+gap)
        cv2.rectangle(img,(x0,y),(x0+bar_w,y+bar_h),(180,180,180),1)
        cv2.rectangle(img,(x0,y),(x0+int(bar_w*val),y+bar_h),(0,200,0),-1)
        label_txt = f"{f}: {int((val)*100)}% ({deg_map[f]:.0f}°)"
        cv2.putText(img, label_txt, (x0+bar_w+10,y+bar_h-3),
                    cv2.FONT_HERSHEY_SIMPLEX,0.55,(255,255,255),1,cv2.LINE_AA)

def euclidean(p1,p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def main():
    # Serial
    ser=None; last_send_ms=0
    if serial is not None:
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
            time.sleep(2.0)
            print(f"[SERIAL] Connected {SERIAL_PORT} @ {SERIAL_BAUD}")
        except Exception as e:
            print(f"[SERIAL] Gagal buka {SERIAL_PORT}: {e}. Lanjut tanpa serial.")

    cap=cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Gagal membuka kamera."); sys.exit(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, model_complexity=1,
                           min_detection_confidence=0.6, min_tracking_confidence=0.6)

    smoother = EmaSmoother(alpha=SMOOTH_ALPHA)
    bounds   = MultiFingerBounds()

    help_text = "Keys: O/[:OPEN  C/]:CLOSE  R:Reset  P:Pause  Q:Quit"
    prev_time=time.time(); paused=False; dynamic_pinch_thresh=None

    # default to middle angles
    send_vals=[90,90,90,90,90]

    while True:
        ok, frame=cap.read()
        if not ok: break
        h,w=frame.shape[:2]

        key=cv2.waitKey(1)&0xFF
        if key==ord('p'): paused=not paused
        elif key==ord('q'): break
        elif key==ord('r'): bounds = MultiFingerBounds()  # reset

        if paused:
            cv2.putText(frame,"PAUSED",(40,90),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,255),4,cv2.LINE_AA)
            cv2.putText(frame,help_text,(40,h-20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(200,200,200),2,cv2.LINE_AA)
            cv2.imshow("Finger -> 5 Servos", frame)
            continue

        rgb=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable=False
        results = hands.process(rgb)
        rgb.flags.writeable=True

        if dynamic_pinch_thresh is None:
            dynamic_pinch_thresh = max(20, int(min(w,h)*0.035))

        if results.multi_hand_landmarks:
            lms = results.multi_hand_landmarks[0]
            handed = results.multi_handedness[0]
            raw_label = handed.classification[0].label
            label = _swap_lr(raw_label) if SWAP_HANDEDNESS_FOR_VIEWER else raw_label

            # draw landmarks
            mp_drawing.draw_landmarks(frame, lms, mp_hands.HAND_CONNECTIONS,
                                      mp_styles.get_default_hand_landmarks_style(),
                                      mp_styles.get_default_hand_connections_style())

            # pinch viz
            tpx=lm_to_pixels(lms.landmark[4], w, h); ipx=lm_to_pixels(lms.landmark[8], w, h)
            dpx=euclidean(tpx,ipx); cx,cy=(tpx[0]+ipx[0])//2,(tpx[1]+ipx[1])//2
            cv2.line(frame,tpx,ipx,(255,255,255),2)
            cv2.circle(frame,tpx,8,(0,0,255),-1); cv2.circle(frame,ipx,8,(0,0,255),-1)
            cv2.circle(frame,(cx,cy),10,(0,255,0) if dpx<dynamic_pinch_thresh else (0,165,255),-1)

            # raw angles
            ang_raw = compute_finger_angles_3d(lms.landmark)
            # smooth raw per finger
            ang_s = {f: smoother.update(label, f"ang_{f}", float(ang_raw[f])) for f in FINGERS}

            # kalibrasi: tekan O/C untuk semua jari
            if key==ord('o') or key==ord('['):
                bounds.set_open_from_angles(label, ang_s)
            elif key==ord('c') or key==ord(']'):
                bounds.set_close_from_angles(label, ang_s)

            # hitung sudut servo (OPEN->0, CLOSE->180)
            servo_deg = {}
            for f in FINGERS:
                deg = bounds.servo_deg(label, f, ang_s[f])  # 0..180
                servo_deg[f] = smoother.update(label, f"servo_{f}", deg)

            # tampil panel
            draw_panel(frame, servo_deg, origin=(10,60), title=f"Servos [{label}] (0=open,180=close)")
            if bounds.has_pair(label):
                oc=bounds.debug_open_close(label)
                txt = "OPEN/CLOSE set"
                cv2.putText(frame, txt, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180,220,255), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, "NO CAL (press O then C)", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,220,255), 2, cv2.LINE_AA)

            # siapkan nilai kirim (Thumb..Pinky, int 0..180)
            send_vals = [ int(np.clip(round(servo_deg[f]),0,180)) for f in FINGERS ]

        # kirim serial (CSV)
        now_ms=int(time.time()*1000)
        if serial is not None and ser is not None and (now_ms-last_send_ms)>=SERIAL_SEND_EVERY_MS:
            line=f"F,{send_vals[0]},{send_vals[1]},{send_vals[2]},{send_vals[3]},{send_vals[4]}\n"
            try: ser.write(line.encode('ascii'))
            except: pass
            last_send_ms=now_ms

        # FPS + help
        now=time.time(); fps=1.0/ (now-prev_time) if now>prev_time else 0.0; prev_time=now
        cv2.putText(frame, f"FPS: {fps:.1f}", (w-140,30), cv2.FONT_HERSHEY_SIMPLEX,0.7,(50,220,50),2)
        cv2.putText(frame, help_text, (10, h-12), cv2.FONT_HERSHEY_SIMPLEX,0.55,(200,200,200),1)
        cv2.imshow("Finger -> 5 Servos", frame)

    hands.close(); cap.release(); cv2.destroyAllWindows()
    if serial is not None and ser is not None:
        try: ser.close()
        except: pass

if __name__=="__main__":
    main()
