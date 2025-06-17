# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['advanced_controller.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=['bleak', 'bleak.backends.winrt', 'bleak.backends.winrt.scanner', 'bleak.backends.winrt.client', 'asyncio', 'threading'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='LEGO-Train-Controller-v2.0',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
