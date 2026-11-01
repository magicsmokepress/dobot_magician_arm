#!/usr/bin/env python3
"""dobot.py - Dobot Magician driver, implementing the Dobot Communication Protocol V1.1.5
(command IDs, framing, checksum and alarm codes from that document). Validated on a production arm.
Reads are safe; home/move/servo/suction actuate hardware - clear the workspace.
Port: /dev/dobot (udev) else /dev/ttyUSB0. pyserial>=3.5."""
import os, struct, time, threading, math
import serial

GET_POSE=10; GET_ALARMS=20; CLEAR_ALARMS=20; SET_HOME=31; SET_HHT_MODE=40; SET_HHT_ENABLE=41; GET_HHT_TRIG=42; SET_SUCTION=62
SET_GRIPPER=63; SET_PTP_JUMP=82; SET_PTP_COMMON=83; SET_PTP_CMD=84; SET_IO_MUX=130; SET_IO_PWM=132; SET_COLOR=137; SET_IR=138; SET_EMOTOR=135
QUEUE_START=240; QUEUE_CLEAR=245; QUEUE_CURIDX=246; SET_WAIT=110
GP3_PWM_EIO=9                                   # GP3 = R-axis servo / claw, PWM=EIO9
LIMITS={"j1":(-90,90),"j2":(0,85),"j3":(-10,90),"j4":(-90,90)}   # real firmware soft-limits
REACH_MM=320
ROUTINE_DIR=os.environ.get("DOBOT_ROUTINE_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),"routines"))
PTP={"JUMP_XYZ":0,"MOVJ_XYZ":1,"MOVL_XYZ":2,"JUMP_ANGLE":3,"MOVJ_ANGLE":4,"MOVL_ANGLE":5}
ALARM_NAMES={0x00:"reset",0x01:"undefined_cmd",0x02:"file_system",0x03:"mcu_fpga_comm",
 0x04:"angle_sensor",0x10:"plan_inv_abnormal",0x11:"plan_inv_workspace",0x12:"plan_inv_limit",
 0x13:"data_repetition",0x14:"arc_param",0x15:"jump_param",0x20:"move_singularity",
 0x21:"move_inv_calc",0x22:"move_inv_limit",0x30:"overspeed_j1",0x31:"overspeed_j2",
 0x32:"overspeed_j3",0x33:"overspeed_j4",0x40:"limit_j1_pos",0x41:"limit_j1_neg",
 0x42:"limit_j2_pos",0x43:"limit_j2_neg",0x44:"limit_j3_pos",0x45:"limit_j3_neg",
 0x46:"limit_j4_pos",0x47:"limit_j4_neg",0x48:"limit_j23_pos",0x49:"limit_j23_neg",
 0x50:"lostep_j1",0x51:"lostep_j2",0x52:"lostep_j3",0x53:"lostep_j4"}

class DobotError(Exception): pass

class Dobot:
    def __init__(self, port="/dev/dobot", baud=115200, timeout=1.0):
        last=None
        for p in (port,"/dev/ttyUSB0"):
            try: self._ser=serial.Serial(p,baud,timeout=timeout); self.port=p; break
            except serial.SerialException as e: last=e
        else: raise DobotError(f"cannot open dobot port: {last}")
        self._lock=threading.Lock()
    @staticmethod
    def _frame(cid,ctrl=0,params=b""):
        pl=bytes([cid,ctrl])+params
        return bytes([0xAA,0xAA,len(pl)])+pl+bytes([(256-(sum(pl)&0xFF))&0xFF])
    def _txn(self,cid,ctrl=0,params=b"",rd=128,settle=0.12):
        with self._lock:
            self._ser.reset_input_buffer(); self._ser.write(self._frame(cid,ctrl,params))
            time.sleep(settle); raw=self._ser.read(rd)
        i=raw.find(b"\xAA\xAA")
        if i<0 or len(raw)<i+3: raise DobotError(f"no frame cmd {cid}: {raw.hex()}")
        ln=raw[i+2]; return raw[i+3:i+3+ln][2:]
    # ---- reads ----
    def get_pose(self):
        p=self._txn(GET_POSE,rd=64)
        return dict(zip(("x","y","z","r","j1","j2","j3","j4"),struct.unpack("<8f",p[:32])))
    def get_alarms(self): return self._txn(GET_ALARMS,rd=64)
    def decode_alarms(self):
        out=[]; 
        for bi,byte in enumerate(self.get_alarms()):
            for bit in range(8):
                if byte>>bit & 1: out.append(ALARM_NAMES.get(bi*8+bit,hex(bi*8+bit)))
        return out
    def clear_alarms(self): self._txn(CLEAR_ALARMS,0x01)
    def _wait_stable(self,timeout,minrun=1.0):
        prev=None; stable=0; t0=time.time()
        while time.time()-t0<timeout:
            time.sleep(0.25)
            try: cur=tuple(self.get_pose().values())
            except DobotError: continue
            stable=stable+1 if (prev and max(abs(a-b) for a,b in zip(cur,prev))<0.4) else 0
            prev=cur
            if stable>=2 and time.time()-t0>minrun: break
    def _wait_queue(self, target_idx, timeout=120):
        t0=time.time()
        while time.time()-t0<timeout:
            r=self._txn(QUEUE_CURIDX,0x00,rd=64)
            cur=struct.unpack("<Q",r[:8])[0] if len(r)>=8 else 0
            if cur>=target_idx: return True
            time.sleep(0.12)
        return False
    # ---- motion ----
    def home(self, wait=True, timeout=60):
        self._txn(CLEAR_ALARMS,0x01); self._txn(QUEUE_CLEAR,0x01); self._txn(QUEUE_START,0x01)
        self._txn(SET_HOME,0x03,struct.pack("<I",0))
        if wait: self._wait_stable(timeout); self.clear_alarms()
        return self.get_pose()
    def _set_ptp_speed(self,vel,acc):
        self._txn(SET_PTP_COMMON,0x01,struct.pack("<ff",float(vel),float(acc)))
    def move_to(self, x,y,z,r=0.0, mode="MOVJ_XYZ", vel=30.0, acc=30.0, wait=True, timeout=40):
        """Cartesian move (mm, r in deg). Conservative speed by default.
        Guards obvious out-of-reach targets; firmware alarms on the rest (we report)."""
        rad=math.hypot(x,y)
        if not (50<=rad<=REACH_MM): raise DobotError(f"target planar radius {rad:.0f}mm out of [50,{REACH_MM}]")
        if not (LIMITS['j4'][0]<=r<=LIMITS['j4'][1]): raise DobotError(f"r={r} out of J4 limit {LIMITS['j4']}")
        self._txn(QUEUE_START,0x01); self._set_ptp_speed(vel,acc)
        self._txn(SET_PTP_CMD,0x03,bytes([PTP[mode]])+struct.pack("<ffff",float(x),float(y),float(z),float(r)))
        if wait: self._wait_stable(timeout)
        al=self.decode_alarms()
        if al: self.clear_alarms()
        return {"pose":self.get_pose(),"alarms":al}
    def move_joint(self, j1,j2,j3,j4, vel=30.0, acc=30.0, wait=True, timeout=40):
        for k,v in zip(("j1","j2","j3","j4"),(j1,j2,j3,j4)):
            lo,hi=LIMITS[k]
            if not lo<=v<=hi: raise DobotError(f"{k}={v} out of soft-limit ({lo},{hi})")
        self._txn(QUEUE_START,0x01); self._set_ptp_speed(vel,acc)
        self._txn(SET_PTP_CMD,0x03,bytes([PTP["MOVJ_ANGLE"]])+struct.pack("<ffff",j1,j2,j3,j4))
        if wait: self._wait_stable(timeout)
        return {"pose":self.get_pose(),"alarms":self.decode_alarms()}
    # ---- end effectors ----
    def suction(self, on):                          # GP1 air pump (cmd 62)
        self._txn(SET_SUCTION,0x01,bytes([1,1 if on else 0]))
    def set_servo_pwm(self, duty_pct, freq_hz=50.0): # GP3/EIO9 claw servo
        duty=max(2.5,min(12.5,float(duty_pct)))
        self._txn(SET_IO_MUX,0x01,bytes([GP3_PWM_EIO,1]))
        self._txn(SET_IO_PWM,0x01,bytes([GP3_PWM_EIO])+struct.pack("<ff",float(freq_hz),duty))
        return duty
    def sweep_servo(self, lo=7.5, hi=12.5, step=1.0, dwell=1.5):
        """SUPERVISED gentle sweep to find an endpoint. Watch the claw; the moment it
        stops moving or the servo strains/buzzes, that duty is the mechanical stop -
        back off 0.5%. Call once for each direction (raise hi or lower lo)."""
        d=lo
        while (d<=hi if hi>=lo else d>=hi):
            print(f"  servo duty={d:.1f}%  (pulse {d/100*20:.2f}ms)"); self.set_servo_pwm(d)
            time.sleep(dwell); d += step if hi>=lo else -step
    def grip(self):    self._txn(SET_GRIPPER,0x01,bytes([1,1]))   # pneumatic claw CLOSE (suck)
    def release(self): self._txn(SET_GRIPPER,0x01,bytes([1,0]))   # pneumatic claw OPEN (blow)
    def pump_off(self):self._txn(SET_GRIPPER,0x01,bytes([0,0]))   # stop pump (do after grip/release settles)
    def set_jump_params(self, height=40.0, z_limit=120.0):
        """JUMP-mode lift profile: rise height mm, capped at z_limit mm (cmd 82)."""
        self._txn(SET_PTP_JUMP,0x01,struct.pack("<ff",float(height),float(z_limit)))
    def jump_to(self, x,y,z, r=0.0, height=40.0, vel=100, acc=100, wait=True, timeout=40):
        """Pick/place: lift, travel over, descend (JUMP_XYZ)."""
        if not (50<=math.hypot(x,y)<=REACH_MM): raise DobotError(f"target {x},{y} out of reach")
        self._txn(QUEUE_START,0x01); self.set_jump_params(height); self._set_ptp_speed(vel,acc)
        self._txn(SET_PTP_CMD,0x03,bytes([PTP["JUMP_XYZ"]])+struct.pack("<ffff",float(x),float(y),float(z),float(r)))
        if wait: self._wait_stable(timeout)
        al=self.decode_alarms()
        if al: self.clear_alarms()
        return {"pose":self.get_pose(),"alarms":al}
    def run_sequence(self, steps, vel=100, acc=100, jump_height=40, timeout=120):
        """Queue a whole path -> firmware flows through it continuously.
        steps: ("move",x,y,z,r) | ("grip",) | ("release",) | ("wait",ms)."""
        self._txn(CLEAR_ALARMS,0x01); self._txn(QUEUE_CLEAR,0x01); self._txn(QUEUE_START,0x01)
        self._set_ptp_speed(vel,acc); self.set_jump_params(jump_height); last=0
        for s in steps:
            k=s[0]
            if k=="move":
                _,x,y,z,r=s
                if not (50<=math.hypot(x,y)<=REACH_MM): raise DobotError(f"waypoint {x},{y} out of reach")
                p=self._txn(SET_PTP_CMD,0x03,bytes([PTP["MOVJ_XYZ"]])+struct.pack("<ffff",float(x),float(y),float(z),float(r)))
            elif k=="jump":
                _,x,y,z,r=s
                if not (50<=math.hypot(x,y)<=REACH_MM): raise DobotError(f"waypoint {x},{y} out of reach")
                p=self._txn(SET_PTP_CMD,0x03,bytes([PTP["JUMP_XYZ"]])+struct.pack("<ffff",float(x),float(y),float(z),float(r)))
            elif k=="grip":    p=self._txn(SET_GRIPPER,0x03,bytes([1,1]))
            elif k=="release": p=self._txn(SET_GRIPPER,0x03,bytes([1,0]))
            elif k=="wait":    p=self._txn(SET_WAIT,0x03,struct.pack("<I",int(s[1])))
            else: continue
            if len(p)>=8: last=struct.unpack("<Q",p[:8])[0]
        ok=self._wait_queue(last,timeout); al=self.decode_alarms()
        if al: self.clear_alarms()
        return {"pose":self.get_pose(),"alarms":al,"completed":ok}
    def set_color(self, on=True, port=1, version=1):
        self._txn(SET_COLOR,0x03,bytes([int(on),port,version]))
    def get_color(self, port=1, version=1):
        """Dobot color sensor (GP2): returns {r,g,b} booleans for detected color."""
        r=self._txn(SET_COLOR,0x00,bytes([port,1,version]),rd=64)
        return {"r":bool(r[0]),"g":bool(r[1]),"b":bool(r[2])} if len(r)>=3 else None
    def set_ir(self, on=True, port=2):
        self._txn(SET_IR,0x02,bytes([int(on),port]))
    def get_ir(self, port=2):
        """IR/photoelectric sensor (GP4): True = object detected."""
        r=self._txn(SET_IR,0x00,bytes([port]),rd=64)
        return bool(r[0]) if r else None
    def conveyor(self, speed_frac=0.0, direction=1, interface=0):
        """Conveyor/stepper (EMotor): speed_frac 0..1, direction +/-1; 0 stops."""
        spd=int(50*speed_frac*(360.0/1.8*10.0*16.0)/(math.pi*36.0)*direction)
        self._txn(SET_EMOTOR,0x03,bytes([interface&1,1])+struct.pack("<i",spd)); return spd
    def teach(self, name=None, max_points=30, timeout=180, settle=0.15):
        """Hand-hold teaching: hold the forearm unlock button, move the arm by hand,
        release to capture each pose. Stops at max_points or timeout. Saves routines/<name>.json."""
        import os, json
        self._txn(SET_HHT_MODE,0x01,bytes([0]))      # trigger on key release
        self._txn(SET_HHT_ENABLE,0x01,bytes([1]))    # enable trigger output
        pts=[]; last=False; t0=time.time()
        try:
            while len(pts)<max_points and time.time()-t0<timeout:
                r=self._txn(GET_HHT_TRIG,0x00,rd=64)
                trig=bool(r[0]) if r else False
                if trig and not last:
                    p=self.get_pose()
                    pts.append({k:round(p[k],2) for k in ("x","y","z","r")})
                    print("  captured #%d: x=%.0f y=%.0f z=%.0f r=%.0f"%(len(pts),p["x"],p["y"],p["z"],p["r"]),flush=True)
                last=trig
                time.sleep(settle)
        finally:
            self._txn(SET_HHT_ENABLE,0x01,bytes([0]))
        if name:
            os.makedirs(ROUTINE_DIR,exist_ok=True)
            json.dump(pts,open(os.path.join(ROUTINE_DIR,name+".json"),"w"),indent=2)
        return pts
    def list_routines(self):
        import os
        return [f[:-5] for f in os.listdir(ROUTINE_DIR)] if os.path.isdir(ROUTINE_DIR) else []
    def playback(self, name, vel=80, acc=80, loops=1):
        import os, json
        pts=json.load(open(os.path.join(ROUTINE_DIR,name+".json")))
        steps=[("move",p["x"],p["y"],p["z"],p.get("r",0)) for p in pts]*int(loops)
        return self.run_sequence(steps,vel=vel,acc=acc)
    def close(self):
        try: self._ser.close()
        except Exception: pass

if __name__=="__main__":
    import sys; d=Dobot(); a=sys.argv[1:]
    print("port:",d.port,"pose:",{k:round(v,1) for k,v in d.get_pose().items()},"alarms:",d.decode_alarms() or "none")
    d.close()
