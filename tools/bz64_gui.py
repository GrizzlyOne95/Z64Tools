#!/usr/bin/env python3
"""
Battlezone 64 extraction GUI.

This UI is a styled front-end for tools/bz64_extract.py. It introspects the
argparse parser from that module so command options stay in sync with CLI.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shlex
import subprocess
import sys
import threading
import tkinter as tk
import tkinter.font as tkfont
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional

try:
    from PIL import Image, ImageTk
except Exception:
    Image = None  # type: ignore[assignment]
    ImageTk = None  # type: ignore[assignment]


# Style constants inspired by Battlezone98Redux_TextureManager.
BZ_BG = "#0a0a0a"
BZ_FG = "#d4d4d4"
BZ_GREEN = "#00ff00"
BZ_DARK_GREEN = "#004400"
BZ_CYAN = "#00ffff"
BZ_DIM_CYAN = "#6aa8a8"

CONFIG_FILE = "bz64_gui_config.json"


def _load_custom_font(font_path: str) -> bool:
    if not os.path.exists(font_path):
        return False
    if os.name != "nt":
        return False
    try:
        import ctypes

        ctypes.windll.gdi32.AddFontResourceExW(font_path, 0x10, 0)
        return True
    except Exception:
        return False


def _pick_font_family(root: tk.Tk) -> str:
    """Choose the best loaded custom family with sensible fallback."""
    try:
        families = list(tkfont.families(root))
    except Exception:
        return "Consolas"
    lower_map = {f.lower(): f for f in families}
    for wanted in ("coulson", "battlemaze", "battle maze", "bzone"):
        for k, v in lower_map.items():
            if wanted in k:
                return v
    return "Consolas"


class ArgField:
    def __init__(self, action: argparse.Action):
        self.action = action
        self.container: Optional[ttk.Frame] = None
        self.value_var: Optional[tk.Variable] = None
        self.entry: Optional[tk.Entry] = None
        self.combo: Optional[ttk.Combobox] = None
        self.check: Optional[ttk.Checkbutton] = None

    def _path_kind(self) -> str:
        d = self.action.dest.lower()
        if "outdir" in d or d.endswith("indir") or d.endswith("dir"):
            return "dir"
        if d in {"rom", "input", "config", "json", "tlut"}:
            return "file_open"
        if d.startswith("out") or d.endswith("_out") or d.endswith("_path"):
            return "file_save"
        return ""

    def _is_bool_flag(self) -> bool:
        return isinstance(
            self.action,
            (
                argparse._StoreTrueAction,
                argparse._StoreFalseAction,
            ),
        )

    def render(
        self,
        parent: tk.Widget,
        default_value: Optional[str] = None,
        wraplength: int = 900,
    ) -> ttk.Frame:
        frame = ttk.Frame(parent)
        self.container = frame
        label_txt = ", ".join(self.action.option_strings) if self.action.option_strings else self.action.dest
        help_txt = self.action.help or ""

        if self._is_bool_flag():
            # store_true defaults False; store_false defaults True.
            initial = bool(self.action.default)
            if default_value is not None:
                initial = default_value.lower() in {"1", "true", "yes", "on"}
            self.value_var = tk.BooleanVar(value=initial)
            self.check = ttk.Checkbutton(frame, text=label_txt, variable=self.value_var)
            self.check.pack(side="top", anchor="w", fill="x", expand=True, padx=4, pady=(1, 0))
            if help_txt:
                ttk.Label(
                    frame,
                    text=help_txt,
                    style="Help.TLabel",
                    wraplength=wraplength,
                    justify="left",
                ).pack(side="top", anchor="w", fill="x", padx=20, pady=(0, 1))
            return frame

        ttk.Label(frame, text=label_txt, style="FieldName.TLabel").pack(side="top", anchor="w", padx=4)
        if help_txt:
            ttk.Label(
                frame,
                text=help_txt,
                style="Help.TLabel",
                wraplength=wraplength,
                justify="left",
            ).pack(side="top", anchor="w", fill="x", padx=4, pady=(0, 1))
        row = ttk.Frame(frame)
        row.pack(side="top", fill="x", padx=4, pady=(0, 1))

        init_txt = default_value if default_value is not None else ""
        if init_txt == "" and self.action.default not in (None, argparse.SUPPRESS):
            init_txt = str(self.action.default)
            if init_txt in ("None",):
                init_txt = ""

        self.value_var = tk.StringVar(value=init_txt)

        if self.action.choices:
            values = [str(x) for x in self.action.choices]
            self.combo = ttk.Combobox(row, textvariable=self.value_var, values=values, state="readonly")
            self.combo.pack(side="left", fill="x", expand=True)
            if init_txt and init_txt in values:
                self.combo.set(init_txt)
            elif values:
                self.combo.set(values[0])
        else:
            self.entry = ttk.Entry(row, textvariable=self.value_var)
            self.entry.pack(side="left", fill="x", expand=True)

        path_kind = self._path_kind()
        if path_kind:
            ttk.Button(row, text="Browse", command=lambda: self._browse(path_kind)).pack(side="left", padx=(6, 0))
        return frame

    def _browse(self, kind: str) -> None:
        if self.value_var is None:
            return
        if kind == "dir":
            val = filedialog.askdirectory()
        elif kind == "file_open":
            val = filedialog.askopenfilename()
        else:
            val = filedialog.asksaveasfilename()
        if val:
            self.value_var.set(val)

    def to_argv(self) -> List[str]:
        if self.value_var is None:
            return []

        # Handle booleans.
        if self._is_bool_flag():
            v = bool(self.value_var.get())  # type: ignore[truthy-bool]
            if isinstance(self.action, argparse._StoreTrueAction):
                return [self.action.option_strings[0]] if v else []
            if isinstance(self.action, argparse._StoreFalseAction):
                return [self.action.option_strings[0]] if not v else []
            return []

        s = str(self.value_var.get()).strip()
        if not s:
            return []

        # Positional arguments do not include the argument name in argv.
        if not self.action.option_strings:
            return [s]
        opt = self.action.option_strings[0]

        # Append actions accept repeated flags; support comma/semicolon/newline list.
        if isinstance(self.action, argparse._AppendAction):
            vals: List[str] = []
            for part in s.replace(";", "\n").splitlines():
                seg = part.strip()
                if seg:
                    vals.append(seg)
            out: List[str] = []
            for v in vals:
                out.extend([opt, v])
            return out

        return [opt, s]


class BZ64GUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Battlezone 64 Extraction Manager")
        self.root.geometry("1280x980")
        self.root.minsize(1100, 760)
        self.root.configure(bg=BZ_BG)

        self.base_dir = pathlib.Path(__file__).resolve().parent
        self.repo_root = self.base_dir.parent
        self.src_dir = self.repo_root / "src"
        self.script_path = self.base_dir / "bz64_extract.py"
        self.python_exe = sys.executable

        font_candidates = [
            self.src_dir / "Coulson.otf",
            pathlib.Path(r"C:\Users\istuart\Documents\GIT\Z64Tools\src\Coulson.otf"),
            self.src_dir / "BattleMaze.ttf",
            pathlib.Path(r"C:\Users\istuart\Documents\GIT\Z64Tools\src\BattleMaze.ttf"),
            self.repo_root / "BZONE.ttf",
            pathlib.Path(r"C:\Users\istuart\Documents\GIT\Battlezone98Redux_TextureManager\BZONE.ttf"),
        ]
        self.custom_font_name = "Consolas"
        for fp in font_candidates:
            _load_custom_font(str(fp))
        self.custom_font_name = _pick_font_family(self.root)

        icon_candidates = [
            self.repo_root / "bzrtex.ico",
            pathlib.Path(r"C:\Users\istuart\Documents\GIT\Battlezone98Redux_TextureManager\bzrtex.ico"),
        ]
        for ico in icon_candidates:
            if ico.exists():
                try:
                    self.root.iconbitmap(str(ico))
                except Exception:
                    pass
                break

        self._setup_styles()
        self.config = self._load_config()

        # Import parser from CLI module.
        sys.path.insert(0, str(self.base_dir))
        try:
            import bz64_extract as cli_mod  # type: ignore
        except Exception as e:
            messagebox.showerror("Import Error", f"Could not import bz64_extract.py:\n{e}")
            raise
        self.cli_mod = cli_mod
        self.parser: argparse.ArgumentParser = cli_mod.build_parser()
        self.subparsers = self._get_subparsers(self.parser)

        self.current_proc: Optional[subprocess.Popen[str]] = None
        self.current_fields: List[ArgField] = []
        self.form_window_id: Optional[int] = None
        self.cmd_preview_full = ""
        self.command_states: Dict[str, Dict[str, str]] = {}
        if isinstance(self.config.get("commands"), dict):
            for k, v in self.config["commands"].items():
                if isinstance(v, dict):
                    self.command_states[str(k)] = {str(kk): str(vv) for kk, vv in v.items()}
        self.builtin_command_defaults = self._build_builtin_command_defaults()
        self._migrate_command_states()

        self._build_ui()
        self._populate_commands()

    def _setup_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("default")
        main_font = (self.custom_font_name, 10)
        bold_font = (self.custom_font_name, 11, "bold")
        small_font = (self.custom_font_name, 9)
        style.configure(".", background=BZ_BG, foreground=BZ_FG, font=main_font, bordercolor=BZ_DARK_GREEN)
        style.configure("TFrame", background=BZ_BG)
        style.configure("TNotebook", background=BZ_BG, borderwidth=0)
        style.configure("TNotebook.Tab", background="#1a1a1a", foreground=BZ_FG, padding=[10, 2])
        style.map("TNotebook.Tab", background=[("selected", BZ_DARK_GREEN)], foreground=[("selected", BZ_GREEN)])
        style.configure("TLabel", background=BZ_BG, foreground=BZ_FG)
        style.configure("FieldName.TLabel", background=BZ_BG, foreground=BZ_FG, font=small_font)
        style.configure("Help.TLabel", background=BZ_BG, foreground="#a6a6a6", font=small_font)
        style.configure("TEntry", fieldbackground="#1a1a1a", foreground=BZ_CYAN, insertcolor=BZ_GREEN)
        style.configure("TButton", background="#1a1a1a", foreground=BZ_FG)
        style.map("TButton", background=[("active", BZ_DARK_GREEN)], foreground=[("active", BZ_GREEN)])
        style.configure("Action.TButton", foreground=BZ_CYAN, font=bold_font)
        style.configure("Success.TButton", foreground=BZ_GREEN, font=bold_font)
        style.configure(
            "Green.Horizontal.TProgressbar",
            troughcolor="#1a1a1a",
            background=BZ_GREEN,
            bordercolor=BZ_DARK_GREEN,
            lightcolor=BZ_GREEN,
            darkcolor=BZ_DARK_GREEN,
        )
        style.configure("TCheckbutton", background=BZ_BG, foreground=BZ_FG, indicatorcolor=BZ_BG)
        style.map("TCheckbutton", indicatorcolor=[("selected", BZ_GREEN)])
        style.configure("TCombobox", fieldbackground="#1a1a1a", foreground=BZ_CYAN, arrowcolor=BZ_GREEN)
        style.map("TCombobox", fieldbackground=[("readonly", "#1a1a1a")], foreground=[("readonly", BZ_CYAN)])

    def _build_ui(self) -> None:
        header = ttk.Frame(self.root)
        header.pack(fill="x", padx=12, pady=(8, 4))
        ttk.Label(
            header,
            text="Battlezone 64 Extraction Manager",
            font=(self.custom_font_name, 16, "bold"),
            foreground=BZ_GREEN,
        ).pack(side="left", anchor="w")
        self._build_banner_frame(header)

        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=12, pady=(0, 3))
        ttk.Label(top, text="Command:").pack(side="left", padx=(0, 8))
        self.cmd_var = tk.StringVar()
        self.cmd_combo = ttk.Combobox(top, textvariable=self.cmd_var, state="readonly")
        self.cmd_combo.pack(side="left", fill="x", expand=True)
        self.cmd_combo.bind("<<ComboboxSelected>>", self._on_command_changed)

        self.run_btn = ttk.Button(top, text="Run", style="Success.TButton", command=self.run_current)
        self.run_btn.pack(side="left", padx=6)
        self.stop_btn = ttk.Button(top, text="Stop", style="Action.TButton", command=self.stop_current)
        self.stop_btn.pack(side="left", padx=6)
        self.copy_btn = ttk.Button(top, text="Copy Cmd", command=self.copy_command)
        self.copy_btn.pack(side="left", padx=6)

        status_row = ttk.Frame(self.root)
        status_row.pack(fill="x", padx=12, pady=(0, 2))
        self.status_var = tk.StringVar(value="Status: idle")
        ttk.Label(
            status_row,
            textvariable=self.status_var,
            foreground=BZ_DIM_CYAN,
            font=(self.custom_font_name, 9, "bold"),
        ).pack(side="left", padx=(0, 8))
        self.progress = ttk.Progressbar(
            status_row,
            mode="indeterminate",
            style="Green.Horizontal.TProgressbar",
            length=280,
        )
        self.progress.pack(side="left", fill="x", expand=True)
        self.stop_btn.state(["disabled"])

        self.cmd_preview_var = tk.StringVar(value="")
        self.cmd_preview_label = tk.Label(
            self.root,
            textvariable=self.cmd_preview_var,
            foreground=BZ_DIM_CYAN,
            background=BZ_BG,
            anchor="w",
            justify="left",
            font=(self.custom_font_name, 7),
            wraplength=1100,
        )
        self.cmd_preview_label.pack(fill="x", padx=12, pady=(0, 2))

        # Form area with scrolling.
        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True, padx=12, pady=3)
        self.canvas = tk.Canvas(outer, background=BZ_BG, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(outer, orient="vertical", command=self.canvas.yview)
        self.form_frame = ttk.Frame(self.canvas)

        self.form_frame.bind(
            "<Configure>",
            lambda _e: self.canvas.configure(scrollregion=self.canvas.bbox("all")),
        )
        self.form_window_id = self.canvas.create_window((0, 0), window=self.form_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        # Log panel.
        ttk.Label(self.root, text="Execution Log:", foreground=BZ_GREEN, font=(self.custom_font_name, 9, "bold")).pack(
            anchor="w", padx=12
        )
        self.log = tk.Text(self.root, height=7, bg="#050505", fg=BZ_FG, insertbackground=BZ_GREEN, font=("Consolas", 8))
        self.log.pack(fill="x", expand=False, padx=12, pady=(1, 8))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_banner_frame(self, parent: tk.Widget) -> None:
        img_candidates = [
            self.src_dir / "rotbd.jpg",
            pathlib.Path(r"C:\Users\istuart\Documents\GIT\Z64Tools\src\rotbd.jpg"),
        ]
        self.banner_image = self._load_banner_image(img_candidates, max_size=(280, 118))
        if self.banner_image is not None:
            banner_card = tk.Frame(
                parent,
                bg="#050505",
                highlightbackground=BZ_GREEN,
                highlightthickness=2,
                bd=0,
            )
            banner_card.pack(side="right", anchor="ne", padx=(12, 0), pady=0)
            tk.Label(banner_card, image=self.banner_image, bg="#050505", bd=0).pack(padx=3, pady=3)
        else:
            tk.Label(
                parent,
                text="ROTBD Banner Missing",
                fg=BZ_DIM_CYAN,
                bg="#050505",
                font=(self.custom_font_name, 9, "bold"),
            ).pack(side="right", anchor="ne", padx=(12, 0), pady=2)

    @staticmethod
    def _load_banner_image(
        candidates: List[pathlib.Path], max_size: tuple[int, int] = (1180, 220)
    ) -> Optional[object]:
        for p in candidates:
            if not p.exists():
                continue
            try:
                if Image is not None and ImageTk is not None:
                    img = Image.open(p)
                    resampling = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
                    img.thumbnail(max_size, resampling)
                    return ImageTk.PhotoImage(img)
                return tk.PhotoImage(file=str(p))
            except Exception:
                continue
        return None

    def _populate_commands(self) -> None:
        names = sorted(self.subparsers.keys())
        self.cmd_combo["values"] = names
        default_cmd = self.config.get("last_command") if isinstance(self.config.get("last_command"), str) else None
        if default_cmd in self.subparsers:
            self.cmd_var.set(default_cmd)
        elif "audio-extract-wav" in self.subparsers:
            self.cmd_var.set("audio-extract-wav")
        elif names:
            self.cmd_var.set(names[0])
        self._render_command_form()

    def _on_command_changed(self, _event: object) -> None:
        self._snapshot_current_command_state()
        self._render_command_form()

    @staticmethod
    def _get_subparsers(parser: argparse.ArgumentParser) -> Dict[str, argparse.ArgumentParser]:
        for a in parser._actions:
            if isinstance(a, argparse._SubParsersAction):
                return {k: v for k, v in a.choices.items() if isinstance(v, argparse.ArgumentParser)}
        return {}

    def _clear_form(self) -> None:
        for child in self.form_frame.winfo_children():
            child.destroy()
        self.current_fields.clear()

    def _form_wraplength(self) -> int:
        width = self.canvas.winfo_width()
        if width <= 1:
            width = self.root.winfo_width() - 24
        return max(360, width - 36)

    def _apply_wraplengths(self) -> None:
        if hasattr(self, "cmd_preview_label") and self.cmd_preview_label.winfo_exists():
            self.cmd_preview_label.configure(wraplength=max(360, self.root.winfo_width() - 32))
        wraplength = self._form_wraplength()

        def _walk(widget: tk.Widget) -> None:
            for child in widget.winfo_children():
                try:
                    if "wraplength" in child.keys() and int(child.cget("wraplength")) > 0:
                        child.configure(wraplength=wraplength)
                except Exception:
                    pass
                _walk(child)

        _walk(self.form_frame)

    def _on_canvas_configure(self, event: tk.Event) -> None:
        if self.form_window_id is not None:
            self.canvas.itemconfigure(self.form_window_id, width=event.width)
        self._apply_wraplengths()

    def _snapshot_current_command_state(self) -> None:
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return
        vals: Dict[str, str] = {}
        for f in self.current_fields:
            if f.value_var is None:
                continue
            vals[f.action.dest] = str(f.value_var.get())
        if vals:
            self.command_states[cmd] = vals

    def _render_command_form(self) -> None:
        self._clear_form()
        cmd = self.cmd_var.get().strip()
        if not cmd or cmd not in self.subparsers:
            return
        subp = self.subparsers[cmd]
        cmd_cfg = self.command_states.get(cmd, {})
        builtin_cfg = self.builtin_command_defaults.get(cmd, {})
        wraplength = self._form_wraplength()
        ttk.Label(
            self.form_frame,
            text=f"{cmd}: {subp.description or subp.format_usage().strip()}",
            foreground=BZ_GREEN,
            font=(self.custom_font_name, 9, "bold"),
            wraplength=wraplength,
            justify="left",
        ).pack(anchor="w", pady=(2, 4), padx=4)

        for action in subp._actions:
            if action.dest == "help":
                continue
            # Skip hidden/internal.
            if action.dest == argparse.SUPPRESS:
                continue
            field = ArgField(action)
            default_val = None
            if isinstance(cmd_cfg, dict) and action.dest in cmd_cfg:
                cfg_val = str(cmd_cfg[action.dest])
                if cfg_val != "":
                    default_val = cfg_val
            if default_val is None and isinstance(builtin_cfg, dict) and action.dest in builtin_cfg:
                default_val = str(builtin_cfg[action.dest])
            row = field.render(self.form_frame, default_value=default_val, wraplength=wraplength)
            row.pack(fill="x", pady=1, padx=2)
            self.current_fields.append(field)
            if field.value_var is not None:
                field.value_var.trace_add("write", lambda *_args: self._update_preview())
        self._apply_wraplengths()
        self._update_preview()

    def _build_builtin_command_defaults(self) -> Dict[str, Dict[str, str]]:
        defaults: Dict[str, Dict[str, str]] = {}
        rom_path = self.repo_root / "Battlezone - Rise of the Black Dogs (USA).z64"
        ini_path = self.repo_root.parent / "N64SoundTools" / "N64SoundListTool" / "Release" / "gameconfigsound.ini"
        out_audio = self.repo_root / "extract_out" / "audio_wav"
        out_midi = self.repo_root / "extract_out" / "midi"

        defaults["audio-extract-wav"] = {
            "rom": str(rom_path),
            "outdir": str(out_audio),
            "bank": "C217B0,C06F08;E05718,C22140",
            "config_ini": (str(ini_path) if ini_path.exists() else ""),
            "game": "Battlezone - Rise of the Black Dogs (U)",
            "sample_rate": "8000",
            "half_sampling_rate": "False",
            "limit": "",
        }
        defaults["audio-export-midi-bz64"] = {
            "rom": str(rom_path),
            "outdir": str(out_midi),
            "start": "0x00BE4118",
            "end": "0x00C054A8",
            "game_name": "Battlezone - Rise of the Black Dogs (U)",
            "limit": "",
            "no_bin": "False",
        }
        return defaults

    def _migrate_command_states(self) -> None:
        """Upgrade older saved GUI values to current Battlezone audio defaults."""
        cfg = self.command_states.get("audio-extract-wav")
        if isinstance(cfg, dict):
            bank_val = str(cfg.get("bank", "")).strip()
            if bank_val:
                parts: List[str] = []
                for seg in bank_val.replace("\n", ";").split(";"):
                    s = seg.strip().strip("[](){} ")
                    if s:
                        parts.append(s.upper())
                if len(parts) == 1 and parts[0] in {"C217B0,C06F08", "E05718,C22140"}:
                    cfg["bank"] = "C217B0,C06F08;E05718,C22140"
            else:
                cfg["bank"] = "C217B0,C06F08;E05718,C22140"

            if not str(cfg.get("game", "")).strip():
                cfg["game"] = "Battlezone - Rise of the Black Dogs (U)"

            cfg["sample_rate"] = "8000"

            if not str(cfg.get("config_ini", "")).strip():
                ini_path = self.repo_root.parent / "N64SoundTools" / "N64SoundListTool" / "Release" / "gameconfigsound.ini"
                if ini_path.exists():
                    cfg["config_ini"] = str(ini_path)

        midi_cfg = self.command_states.get("audio-export-midi-bz64")
        if isinstance(midi_cfg, dict):
            if not str(midi_cfg.get("start", "")).strip():
                midi_cfg["start"] = "0x00BE4118"
            if not str(midi_cfg.get("end", "")).strip():
                midi_cfg["end"] = "0x00C054A8"
            if not str(midi_cfg.get("game_name", "")).strip():
                midi_cfg["game_name"] = "Battlezone - Rise of the Black Dogs (U)"

    def _build_argv(self) -> List[str]:
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return []
        argv: List[str] = [self.python_exe, str(self.script_path), cmd]
        for f in self.current_fields:
            argv.extend(f.to_argv())
        return argv

    def _update_preview(self) -> None:
        argv = self._build_argv()
        if not argv:
            self.cmd_preview_full = ""
            self.cmd_preview_var.set("")
            return
        self.cmd_preview_full = " ".join(shlex.quote(x) for x in argv)
        preview = self.cmd_preview_full
        if len(preview) > 260:
            head = 165
            tail = 85
            preview = f"{preview[:head]} ... {preview[-tail:]}"
        self.cmd_preview_var.set(preview)

    def _append_log(self, msg: str) -> None:
        self.log.insert("end", msg + ("\n" if not msg.endswith("\n") else ""))
        self.log.see("end")

    def _on_mousewheel(self, event: tk.Event) -> None:
        if not self.canvas.winfo_exists():
            return
        delta = -1 if event.delta > 0 else 1
        self.canvas.yview_scroll(delta, "units")

    def run_current(self) -> None:
        if self.current_proc is not None:
            messagebox.showwarning("Busy", "A command is already running.")
            return
        argv = self._build_argv()
        if not argv:
            messagebox.showerror("Invalid", "No command selected.")
            return
        self._update_preview()
        self._append_log("> " + " ".join(shlex.quote(x) for x in argv))
        self._set_running_state(True)

        def worker() -> None:
            try:
                self.current_proc = subprocess.Popen(
                    argv,
                    cwd=str(self.repo_root),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    universal_newlines=True,
                    bufsize=1,
                )
                assert self.current_proc.stdout is not None
                for line in self.current_proc.stdout:
                    self.root.after(0, lambda l=line: self._append_log(l.rstrip("\n")))
                rc = self.current_proc.wait()
                self.root.after(0, lambda: self._append_log(f"[exit={rc}]"))
            except Exception as e:
                self.root.after(0, lambda: self._append_log(f"[error] {e}"))
            finally:
                self.root.after(0, self._on_process_finished)

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def stop_current(self) -> None:
        if self.current_proc is None:
            return
        try:
            self.current_proc.terminate()
            self._append_log("[terminated]")
        except Exception as e:
            self._append_log(f"[terminate failed] {e}")

    def copy_command(self) -> None:
        self._update_preview()
        txt = self.cmd_preview_full
        if not txt:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(txt)
        self._append_log("[copied command]")

    def _set_running_state(self, running: bool) -> None:
        if running:
            self.status_var.set("Status: running...")
            self.progress.start(12)
            self.run_btn.state(["disabled"])
            self.stop_btn.state(["!disabled"])
            return
        self.status_var.set("Status: idle")
        self.progress.stop()
        self.progress.configure(value=0)
        self.run_btn.state(["!disabled"])
        self.stop_btn.state(["disabled"])

    def _on_process_finished(self) -> None:
        self.current_proc = None
        self._set_running_state(False)

    def _load_config(self) -> Dict[str, object]:
        p = self.repo_root / CONFIG_FILE
        if not p.exists():
            return {}
        try:
            return json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            return {}

    def _save_config(self) -> None:
        self._snapshot_current_command_state()
        cfg: Dict[str, object] = {}
        cfg["last_command"] = self.cmd_var.get().strip()
        cfg["commands"] = self.command_states
        self.config = cfg
        try:
            (self.repo_root / CONFIG_FILE).write_text(json.dumps(cfg, indent=2), encoding="utf-8")
        except Exception:
            pass

    def on_close(self) -> None:
        self._save_config()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    app = BZ64GUI(root)
    # Keep command preview up-to-date when any field changes.
    def periodic_preview() -> None:
        app._update_preview()
        root.after(300, periodic_preview)

    root.after(300, periodic_preview)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
