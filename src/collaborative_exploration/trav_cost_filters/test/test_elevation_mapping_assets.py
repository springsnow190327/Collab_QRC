from pathlib import Path


def test_elevation_mapping_core_assets_are_vendored():
    pkg_root = (
        Path(__file__).resolve().parents[3]
        / "vendor"
        / "elevation_mapping_cupy"
        / "elevation_mapping_cupy"
    )
    core_dir = pkg_root / "config" / "core"

    expected = [
        core_dir / "core_param.yaml",
        core_dir / "plugin_config.yaml",
        core_dir / "weights.dat",
    ]

    missing = [str(path.relative_to(pkg_root)) for path in expected if not path.is_file()]
    assert missing == []
