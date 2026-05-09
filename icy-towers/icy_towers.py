#!/usr/bin/env python3
"""
ICY TOWERS
Climb the icy tower as high as you can!
Controls: A/D or Left/Right to move  |  Space / W / Up to jump  |  Esc for menu
"""

import pygame
import random
import sys
import math
import json
import os

pygame.init()

# ── Window ────────────────────────────────────────────────────
W, H = 400, 600
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Icy Towers")
clock = pygame.time.Clock()
FPS = 60

# ── Fonts ─────────────────────────────────────────────────────
F_BIG  = pygame.font.SysFont("Arial", 52, bold=True)
F_MED  = pygame.font.SysFont("Arial", 30, bold=True)
F_SML  = pygame.font.SysFont("Arial", 20)
F_TINY = pygame.font.SysFont("Arial", 14)

# ── Physics ───────────────────────────────────────────────────
GRAV       = 0.55
MAX_FALL   = 18.0
JUMP_BASE  = -13.0
JUMP_BONUS = 0.35   # running speed adds to jump height
ACCEL      = 0.85
MAX_SPD    = 7.0
ICE_FRIC   = 0.80   # icy ground friction (< 1 = slides)
AIR_FRIC   = 0.97

# ── Platform config ───────────────────────────────────────────
P_H        = 14     # platform height in pixels
FLOOR_GAP  = 80     # base vertical gap between floors
FLOOR_VAR  = 20     # random variation in gap
START_W    = 220    # ground platform width
MIN_W      = 42
MAX_W      = 180

# ── Score ─────────────────────────────────────────────────────
BASE_PTS   = 100

# ── Colors ────────────────────────────────────────────────────
SKY1 = (  8,  18,  55)   # sky gradient top
SKY2 = ( 20,  65, 145)   # sky gradient bottom
PC1  = ( 65, 168, 228)   # platform body
PC2  = (175, 222, 255)   # platform top shine
WH   = (255, 255, 255)
BK   = (  0,   0,   0)
GD   = (255, 215,   0)
RD   = (220,  55,  55)
CY   = ( 95, 210, 255)
LB   = (155, 215, 255)
YW   = (255, 200,  20)
DR   = (255,  80,  80)
PB   = (255, 150,  60)   # player body (orange)
PH2  = (255, 198, 135)   # player head (skin)
PSC  = (210,  40,  40)   # player scarf (red)
PSH  = ( 38,  38,  65)   # player shoes (dark)
PE   = ( 18,  18,  38)   # player eye

HS_FILE = os.path.join(os.path.expanduser("~"), ".icy_towers.json")


# ── Helpers ───────────────────────────────────────────────────
def load_hs():
    try:
        with open(HS_FILE) as f:
            return json.load(f).get("hs", 0)
    except Exception:
        return 0


def save_hs(score):
    try:
        with open(HS_FILE, "w") as f:
            json.dump({"hs": score}, f)
    except Exception:
        pass


# ── Pre-render gradient background ───────────────────────────
bg = pygame.Surface((W, H))
for _y in range(H):
    t = _y / H
    c = tuple(int(SKY1[i] + (SKY2[i] - SKY1[i]) * t) for i in range(3))
    pygame.draw.line(bg, c, (0, _y), (W, _y))


# ─────────────────────────────────────────────────────────────
class Snowflake:
    def __init__(self, initial=False):
        self.x = random.uniform(0, W)
        self.y = random.uniform(0, H) if initial else -4.0
        self.r = random.uniform(1.2, 3.2)
        self.spd = random.uniform(0.5, 1.8)
        self.drift = random.uniform(-0.3, 0.3)
        self.alpha = random.randint(90, 200)

    def update(self, extra_spd=0.0):
        self.y += self.spd + extra_spd
        self.x += self.drift
        if self.x < 0:
            self.x = float(W)
        elif self.x > W:
            self.x = 0.0
        if self.y > H + 6:
            self.y = -4.0
            self.x = random.uniform(0, W)

    def draw(self, surf):
        sz = int(self.r * 2 + 2)
        s = pygame.Surface((sz, sz), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 255, 255, self.alpha), (sz // 2, sz // 2), int(self.r))
        surf.blit(s, (int(self.x - self.r), int(self.y - self.r)))


# ─────────────────────────────────────────────────────────────
class Platform:
    def __init__(self, x, y, w, floor=0):
        self.x = x
        self.y = y
        self.w = w
        self.floor = floor
        self.shimmer_off = random.uniform(0, math.pi * 2)

    def draw(self, surf, cam_y):
        sy = self.y - cam_y
        if sy > H + 20 or sy + P_H < -6:
            return

        # Drop shadow
        shad = pygame.Surface((self.w, 5), pygame.SRCALPHA)
        shad.fill((0, 0, 0, 38))
        surf.blit(shad, (self.x, sy + P_H))

        # Main body
        pygame.draw.rect(surf, PC1, (self.x, sy + 3, self.w, P_H - 3), border_radius=3)

        # Top ice surface
        pygame.draw.rect(surf, PC2, (self.x, sy, self.w, 5), border_radius=3)

        # Animated shimmer highlight
        t = pygame.time.get_ticks() * 0.0014 + self.shimmer_off
        sv = math.sin(t)
        if sv > 0.15:
            px = self.x + int((sv - 0.15) / 0.85 * max(1, self.w - 16))
            ss = pygame.Surface((16, 3), pygame.SRCALPHA)
            ss.fill((255, 255, 255, int(sv * 100)))
            surf.blit(ss, (px, sy + 1))

        # Floor label every 10 floors
        if self.floor > 0 and self.floor % 10 == 0:
            lbl = F_TINY.render(f"Floor {self.floor}", True, WH)
            surf.blit(lbl, (self.x + 3, sy + 2))


# ─────────────────────────────────────────────────────────────
class Player:
    PW = 22
    PHT = 32   # height

    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.on_ground = False
        self.facing = 1       # 1 = right, -1 = left
        self.walk_t = 0.0     # walking animation counter
        self.air_t = 0.0      # airtime (0–1) for stretch anim

    def update(self, platforms):
        keys = pygame.key.get_pressed()

        # Horizontal movement
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            self.vx -= ACCEL
            self.facing = -1
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            self.vx += ACCEL
            self.facing = 1

        self.vx = max(-MAX_SPD, min(MAX_SPD, self.vx))
        self.vx *= ICE_FRIC if self.on_ground else AIR_FRIC

        # Gravity
        self.vy = min(self.vy + GRAV, MAX_FALL)

        # Move X with screen wrap
        self.x += self.vx
        if self.x + self.PW < 0:
            self.x = float(W)
        elif self.x > W:
            self.x = float(-self.PW)

        # Move Y and resolve platform collisions
        prev_bottom = self.y + self.PHT
        self.y += self.vy
        self.on_ground = False

        if self.vy >= 0:  # only land when falling
            curr_bottom = self.y + self.PHT
            for p in platforms:
                if (prev_bottom <= p.y + 2 and
                        curr_bottom >= p.y and
                        self.x + self.PW - 3 >= p.x and
                        self.x + 3 <= p.x + p.w):
                    self.y = float(p.y - self.PHT)
                    self.vy = 0.0
                    self.on_ground = True
                    break

        # Animation timers
        moving = keys[pygame.K_LEFT] or keys[pygame.K_a] or keys[pygame.K_RIGHT] or keys[pygame.K_d]
        if moving and self.on_ground:
            self.walk_t += abs(self.vx) * 0.12
        if self.on_ground:
            self.air_t = max(0.0, self.air_t - 0.2)
        else:
            self.air_t = min(1.0, self.air_t + 0.15)

    def jump(self):
        if self.on_ground:
            # Running fast gives a higher jump (key Icy Towers mechanic!)
            bonus = abs(self.vx) * JUMP_BONUS
            self.vy = JUMP_BASE - bonus
            self.on_ground = False
            self.air_t = 1.0
            return True
        return False

    def draw(self, surf, cam_y):
        sx = int(self.x)
        sy = int(self.y) - cam_y
        if sy > H + 50 or sy + self.PHT < -50:
            return

        leg = math.sin(self.walk_t) * 5 if self.on_ground else 0.0
        stretch = int(self.air_t * 3)

        # Shoes
        pygame.draw.rect(surf, PSH, (sx + 1,             sy + self.PHT - 9 + int(leg),  9, 9), border_radius=2)
        pygame.draw.rect(surf, PSH, (sx + self.PW - 10,  sy + self.PHT - 9 - int(leg),  9, 9), border_radius=2)

        # Body (stretches when airborne)
        body_y = sy + 12 - stretch
        body_h = 16 + stretch * 2
        pygame.draw.rect(surf, PB, (sx + 2, body_y, self.PW - 4, body_h), border_radius=4)

        # Scarf
        pygame.draw.rect(surf, PSC, (sx + 2, sy + 15, self.PW - 4, 4), border_radius=2)

        # Arms (swing opposite to legs)
        arm_s = -leg * 0.5
        pygame.draw.rect(surf, PB, (sx - 3, sy + 13 + int(arm_s),  6, 10), border_radius=2)
        pygame.draw.rect(surf, PB, (sx + self.PW - 3, sy + 13 - int(arm_s), 6, 10), border_radius=2)

        # Head
        hx, hy = sx + self.PW // 2, sy + 8
        pygame.draw.circle(surf, PH2, (hx, hy), 9)

        # Eye (looks in direction of movement)
        pygame.draw.circle(surf, PE, (hx + self.facing * 4, hy - 1), 2)

        # Little winter hat
        pygame.draw.line(surf, (20, 40, 110), (hx - 7, hy - 8), (hx + 7, hy - 8), 3)
        pts = [(hx - 5, hy - 8), (hx + 5, hy - 8), (hx, hy - 17)]
        pygame.draw.polygon(surf, (28, 55, 135), pts)


# ─────────────────────────────────────────────────────────────
class IcyTowers:
    def __init__(self):
        self.hs = load_hs()
        self.snow = [Snowflake(initial=True) for _ in range(55)]
        self.state = "menu"
        self._new_game()

    # ── Game init ─────────────────────────────────────────────
    def _new_game(self):
        # World coordinates: y increases downward; higher floors have smaller y.
        # cam_y = world-y at the top of the screen; screen_y = world_y - cam_y.
        self.cam_y = 0
        self.scroll_spd = 0.4
        self.elapsed_ms = 0

        self.score = 0
        self.best_floor = 0
        self.combo = 0
        self.combo_timer = 0
        self.last_floor_hit = -1

        self.platforms = []
        self._gen_initial()

        gp = self.platforms[0]
        px = gp.x + gp.w // 2 - Player.PW // 2
        self.player = Player(float(px), float(gp.y - Player.PHT))

    # ── Platform generation ───────────────────────────────────
    def _plat_width(self, floor):
        t = min(floor / 50.0, 1.0)
        w = int(START_W * (1 - t) + MIN_W * t)
        w += random.randint(-15, 15)
        return max(MIN_W, min(MAX_W, w))

    def _gen_initial(self):
        ground_y = H - 80
        self.platforms.append(Platform(W // 2 - START_W // 2, ground_y, START_W, 0))
        floor, y = 1, ground_y - FLOOR_GAP
        while y > -H * 2:
            w = self._plat_width(floor)
            x = random.randint(8, W - w - 8)
            self.platforms.append(Platform(x, y, w, floor))
            floor += 1
            y -= FLOOR_GAP + random.randint(-FLOOR_VAR, FLOOR_VAR)
        self.next_floor = floor
        self.next_y = y

    def _gen_more(self):
        need = self.cam_y - H * 1.5
        while self.next_y > need:
            w = self._plat_width(self.next_floor)
            x = random.randint(8, W - w - 8)
            self.platforms.append(Platform(x, self.next_y, w, self.next_floor))
            self.next_floor += 1
            gap = FLOOR_GAP + random.randint(-FLOOR_VAR, FLOOR_VAR)
            gap += min(self.next_floor // 8, 30)  # gaps widen on higher floors
            self.next_y -= gap

    def _cleanup(self):
        cutoff = self.cam_y + H + 120
        self.platforms = [p for p in self.platforms if p.y < cutoff]

    # ── Camera / scrolling ────────────────────────────────────
    def _scroll(self, dt_ms):
        # Auto-scroll speed increases over time
        self.scroll_spd = min(0.4 + self.elapsed_ms * 0.00007, 3.0)
        self.cam_y -= self.scroll_spd * dt_ms / 16.0  # cam moves up → content falls

        # Smoothly follow player when they're in the upper 38% of screen
        target = self.player.y - H * 0.38
        if target < self.cam_y:
            self.cam_y += (target - self.cam_y) * 0.10

    # ── Scoring & combo ───────────────────────────────────────
    def _update_score(self):
        if not self.player.on_ground:
            return
        py_bot = self.player.y + Player.PHT
        for p in self.platforms:
            if (abs(py_bot - p.y) < 5 and
                    self.player.x + Player.PW - 3 >= p.x and
                    self.player.x + 3 <= p.x + p.w):
                if p.floor > self.best_floor:
                    delta = p.floor - self.best_floor
                    # Combo: new floor reached quickly from a different floor
                    if p.floor != self.last_floor_hit and self.combo_timer > 0:
                        self.combo += 1
                    else:
                        self.combo = 1
                    self.last_floor_hit = p.floor
                    self.combo_timer = 90
                    mult = 1.0 + (self.combo - 1) * 0.5
                    self.score += int(delta * BASE_PTS * mult)
                    self.best_floor = p.floor
                break

        if self.combo_timer > 0:
            self.combo_timer -= 1
        else:
            self.combo = 0

    # ── Event handling ────────────────────────────────────────
    def handle_event(self, ev):
        if ev.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if ev.type != pygame.KEYDOWN:
            return

        if self.state == "menu":
            self.state = "playing"
            self._new_game()

        elif self.state == "playing":
            if ev.key in (pygame.K_SPACE, pygame.K_UP, pygame.K_w):
                self.player.jump()
            elif ev.key == pygame.K_ESCAPE:
                self.state = "menu"

        elif self.state == "gameover":
            if ev.key in (pygame.K_r, pygame.K_RETURN):
                self.state = "playing"
                self._new_game()
            elif ev.key == pygame.K_ESCAPE:
                self.state = "menu"

    # ── Update ────────────────────────────────────────────────
    def update(self):
        if self.state != "playing":
            return
        dt = clock.get_time()
        self.elapsed_ms += dt

        self.player.update(self.platforms)
        self._scroll(dt)
        self._gen_more()
        self._cleanup()
        self._update_score()

        # Death check: player fell below screen
        if self.player.y - self.cam_y > H + 60:
            if self.score > self.hs:
                self.hs = self.score
                save_hs(self.hs)
            self.state = "gameover"

        for sf in self.snow:
            sf.update(self.scroll_spd * 0.25)

    # ── Draw ──────────────────────────────────────────────────
    def draw(self):
        screen.blit(bg, (0, 0))

        for sf in self.snow:
            sf.draw(screen)

        if self.state == "menu":
            self._draw_menu()
        elif self.state == "playing":
            self._draw_game()
            self._draw_hud()
        elif self.state == "gameover":
            self._draw_game()
            self._draw_hud()
            self._draw_gameover()

        pygame.display.flip()

    def _draw_game(self):
        cam = int(self.cam_y)
        for p in self.platforms:
            p.draw(screen, cam)
        self.player.draw(screen, cam)

    def _draw_hud(self):
        # Score top-left
        sc = F_MED.render(f"{self.score:,}", True, WH)
        screen.blit(sc, (10, 8))

        # Floor count
        fl = F_SML.render(f"Floor {self.best_floor}", True, LB)
        screen.blit(fl, (10, 42))

        # High score top-right
        hs = F_SML.render(f"Best: {self.hs:,}", True, GD)
        screen.blit(hs, hs.get_rect(topright=(W - 8, 8)))

        # Combo (centre, flashing)
        if self.combo >= 2:
            if (pygame.time.get_ticks() // 175) % 2 == 0:
                ct = F_MED.render(f"  x{self.combo} COMBO!  ", True, YW)
                screen.blit(ct, ct.get_rect(centerx=W // 2, top=10))

        # Speed warning (bottom) — becomes visible as scroll accelerates
        if self.scroll_spd > 1.2:
            wt = F_SML.render("▲  FASTER  ▲", True, DR)
            screen.blit(wt, wt.get_rect(centerx=W // 2, bottom=H - 8))

        # Danger gradient at bottom (red glow when close to death)
        danger_h = min(80, int((self.scroll_spd - 0.8) * 50))
        if danger_h > 0:
            dov = pygame.Surface((W, danger_h), pygame.SRCALPHA)
            for dy in range(danger_h):
                a = int(60 * (1 - dy / danger_h))
                pygame.draw.line(dov, (255, 40, 40, a), (0, dy), (W, dy))
            screen.blit(dov, (0, H - danger_h))

    def _draw_menu(self):
        t1 = F_BIG.render("ICY", True, WH)
        t2 = F_BIG.render("TOWERS", True, CY)
        screen.blit(t1, t1.get_rect(centerx=W // 2, centery=155))
        screen.blit(t2, t2.get_rect(centerx=W // 2, centery=215))

        # Decorative ice lines
        for yi in (152, 265):
            pygame.draw.line(screen, LB, (20, yi), (W - 20, yi), 1)

        sub = F_SML.render("Climb the tower. Reach new heights!", True, LB)
        screen.blit(sub, sub.get_rect(centerx=W // 2, centery=285))

        for i, (k, v) in enumerate([
            ("←→ / A D", "Move (icy!)"),
            ("Space / Up / W", "Jump  (run for higher jumps)"),
        ]):
            kt = F_SML.render(k, True, GD)
            vt = F_SML.render(f"  —  {v}", True, WH)
            rx = W // 2 - (kt.get_width() + vt.get_width()) // 2
            screen.blit(kt, (rx, 338 + i * 32))
            screen.blit(vt, (rx + kt.get_width(), 338 + i * 32))

        if (pygame.time.get_ticks() // 500) % 2 == 0:
            st = F_SML.render("Press any key to start", True, GD)
            screen.blit(st, st.get_rect(centerx=W // 2, centery=440))

        if self.hs > 0:
            ht = F_SML.render(f"High Score: {self.hs:,}", True, GD)
            screen.blit(ht, ht.get_rect(centerx=W // 2, centery=535))

    def _draw_gameover(self):
        ov = pygame.Surface((W, H), pygame.SRCALPHA)
        ov.fill((0, 0, 25, 172))
        screen.blit(ov, (0, 0))

        go = F_BIG.render("GAME OVER", True, RD)
        screen.blit(go, go.get_rect(centerx=W // 2, centery=175))

        sc = F_MED.render(f"Score: {self.score:,}", True, WH)
        screen.blit(sc, sc.get_rect(centerx=W // 2, centery=258))

        fl = F_SML.render(f"Reached floor {self.best_floor}", True, LB)
        screen.blit(fl, fl.get_rect(centerx=W // 2, centery=298))

        if self.score > 0 and self.score >= self.hs:
            nh = F_MED.render("★ NEW HIGH SCORE! ★", True, GD)
            screen.blit(nh, nh.get_rect(centerx=W // 2, centery=345))
        else:
            bt = F_SML.render(f"Best: {self.hs:,}", True, GD)
            screen.blit(bt, bt.get_rect(centerx=W // 2, centery=345))

        if (pygame.time.get_ticks() // 600) % 2 == 0:
            rt = F_SML.render("Enter / R = Restart     Esc = Menu", True, WH)
            screen.blit(rt, rt.get_rect(centerx=W // 2, centery=432))


# ── Entry point ───────────────────────────────────────────────
def main():
    game = IcyTowers()
    while True:
        for event in pygame.event.get():
            game.handle_event(event)
        game.update()
        game.draw()
        clock.tick(FPS)


if __name__ == "__main__":
    main()
