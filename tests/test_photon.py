import pathlib
import sys

import pytest

from real2sim.tactile import photon

GOOD = {
    "schema_version": "1.0",
    "sensor": "photon",
    "outputs": ["depth", "rgb"],
    "declared_synthetic": True,
    "stimulus": {"type": "gaussian", "amplitude_m": 4e-4, "sigma_m": 3e-3},
}


def test_render_config_accepts_valid():
    cfg = photon.validate_render_config(GOOD)
    assert cfg["outputs"] == ["depth", "rgb"]
    assert cfg["stimulus"]["amplitude_m"] == pytest.approx(4e-4)
    assert cfg["device"] == "cuda"


def test_render_config_rejects_bad_inputs():
    with pytest.raises(ValueError):
        photon.validate_render_config({**GOOD, "schema_version": "2.0"})
    with pytest.raises(ValueError):
        photon.validate_render_config({**GOOD, "sensor": "gsmini"})
    with pytest.raises(ValueError):
        photon.validate_render_config({**GOOD, "outputs": ["sonar"]})
    with pytest.raises(ValueError):
        photon.validate_render_config({**GOOD, "outputs": []})
    # The synthetic stimulus must be declared: simulated tactile must not pass as real contact
    with pytest.raises(ValueError):
        photon.validate_render_config({k: v for k, v in GOOD.items() if k != "declared_synthetic"})
    with pytest.raises(ValueError):
        photon.validate_render_config({**GOOD, "stimulus": {"type": "gaussian", "amplitude_m": -1, "sigma_m": 3e-3}})
    with pytest.raises(ValueError):
        photon.validate_render_config({**GOOD, "stimulus": {"type": "poker", "amplitude_m": 1e-3, "sigma_m": 3e-3}})


def test_find_bundle_matches_vendor_layout(tmp_path):
    assert photon.find_bundle() is None
    assert photon.find_bundle(tmp_path) is None
    bundle = tmp_path / "third_party" / "xense" / "isaac_xensesim4.5" / "pip_prebundle" / "xensim"
    bundle.mkdir(parents=True)
    assert photon.find_bundle(tmp_path) == bundle


def test_find_bundle_matches_committed_layout(tmp_path):
    # Since tacsim d5ce700 the vendor bundle is committed at third_party/xense_photon/, one
    # level shallower than the old hand-deployed layout: the `**` in BUNDLE_GLOB matches zero
    # directories, so both layouts resolve without a code change. Separate tmp_path from the
    # test above because find_bundle returns only the first sorted match.
    bundle = tmp_path / "third_party" / "xense_photon" / "pip_prebundle" / "xensim"
    bundle.mkdir(parents=True)
    assert photon.find_bundle(tmp_path) == bundle


def test_tacsim_root_from_module_derives_checkout_root():
    root = photon.tacsim_root_from_module("/opt/tacsim/tacsim/__init__.py")
    assert root == pathlib.Path("/opt/tacsim")
    # Anything that is not an imported tacsim package -- including the probe's
    # 'missing: ModuleNotFoundError' sentinel -- must not be mistaken for a root.
    assert photon.tacsim_root_from_module("missing: ModuleNotFoundError") is None
    assert photon.tacsim_root_from_module(None) is None
    assert photon.tacsim_root_from_module("/opt/other/__init__.py") is None


def test_runtime_check_reports_without_raising():
    report = photon.check_photon_runtime(sys.executable)
    assert set(report["checks"]) == {
        "tacsim_importable", "tacsim_repo_root_found", "xense_bundle_deployed",
        "python_is_3.10", "cffi_importable", "pyudev_importable", "cuda_available",
    }
    # Every check must stay a strict bool: `ready` reads them for truthiness, so a
    # string would pass silently.
    assert all(isinstance(v, bool) for v in report["checks"].values())
    assert report["tacsim_root"] is None or isinstance(report["tacsim_root"], str)
    assert report["tacsim_root_kind"] in (None, "git_checkout", "directory")
    assert isinstance(report["ready"], bool)
    missing = photon.check_photon_runtime(sys.executable, tacsim_root=pathlib.Path("/nonexistent"))
    assert missing["ready"] is False
    assert missing["checks"]["xense_bundle_deployed"] is False
    assert missing["checks"]["tacsim_repo_root_found"] is False


class _FakeTensor:
    def __init__(self, array):
        import numpy as np
        self._array = np.asarray(array)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._array


class _FakeFrame:
    def __init__(self, **channels):
        vars(self).update(channels)


def test_save_requested_outputs_skips_unrequested_channels(tmp_path):
    import numpy as np
    from real2sim.tactile.photon_worker import save_requested_outputs

    # depth_m is None when the backend was not asked for depth; asking only for marker_flow must not read depth
    frame = _FakeFrame(depth_m=None, rgb=None, marker_flow=_FakeTensor(np.zeros((2, 220, 2))))
    files = save_requested_outputs(frame, ["marker_flow"], tmp_path)
    assert list(files) == ["marker_flow.npy"]
    assert (tmp_path / "marker_flow.npy").is_file()
    assert not (tmp_path / "depth_m.npy").exists()

    # depth was requested but the backend produced none: raise a clear error, not silence or an AttributeError
    with pytest.raises(RuntimeError, match="depth"):
        save_requested_outputs(frame, ["depth"], tmp_path)
    with pytest.raises(RuntimeError, match="rgb"):
        save_requested_outputs(frame, ["rgb"], tmp_path)
