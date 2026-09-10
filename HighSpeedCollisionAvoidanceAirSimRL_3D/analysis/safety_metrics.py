"""Shared safety-signal estimator (dual-use).

Given the FUSED depth image the policy actually observes (120x120, single
channel; pixel value = 255 / depth, so HIGH intensity == CLOSE obstacle),
estimate two scalars:

    P_coll  : collision-probability proxy in [0, 1]   (high = dangerous)
    Delta_r : forward clearance proxy, normalized to [0, 1]  (1 = far / safe)

THE SAME estimator is used in two decoupled places, which is the whole point
of the design:

  * Training (in-loop): P_coll and Delta_r are fed to the policy as an
    extra observation key and P_coll drives a chance-constraint reward penalty.
    -> safety is *prescriptively* coupled into the learned policy.

  * Eval-time (logged episodes): the identical estimator is run on the eval
    rollouts, on a path that is completely separate from the policy/training
    graph, purely to LOG and VERIFY (calibration, binding, ablation).

So the safety signal that the policy was trained against is exactly the signal
the offline analysis calibrates — "in-loop coupled safety, independently
verified". The estimator is intentionally simple and fully parameterized by
analysis/params.yml (no magic numbers here).
"""
import numpy as np


def _center_roi(img2d, roi_h, roi_w):
    h, w = img2d.shape[:2]
    rh = min(int(roi_h), h)
    rw = min(int(roi_w), w)
    y0 = (h - rh) // 2
    x0 = (w - rw) // 2
    return img2d[y0:y0 + rh, x0:x0 + rw]


def estimate(fused_image, params):
    """Return (P_coll, Delta_r) for a fused depth image.

    fused_image : ndarray, (120,120), (120,120,1) or (120,120,3). For the
                  3-channel naive stack baseline it is averaged over channels.
    params      : the dict from analysis.params.load_params().
    """
    roi_cfg = (params or {}).get("roi", {})
    roi_h = roi_cfg.get("roi_h", 40)
    roi_w = roi_cfg.get("roi_w", 40)
    intensity_max = float(roi_cfg.get("intensity_max", 255.0))
    top_k_frac = float(roi_cfg.get("top_k_frac", 0.10))
    r_max = float(roi_cfg.get("r_max", 20.0))

    img = np.asarray(fused_image, dtype=np.float64)
    img = np.squeeze(img)                 # (120,120,1) -> (120,120)
    if img.ndim == 3:                     # 3-channel stack -> mean over channels
        img = img.mean(axis=2)
    if img.ndim != 2:
        return 0.0, 1.0

    roi = _center_roi(img, roi_h, roi_w).ravel()
    if roi.size == 0:
        return 0.0, 1.0

    # P_coll: average intensity of the nearest (highest-intensity) ROI pixels,
    # normalized to [0,1]. Using the top-k fraction (instead of the single max)
    # is robust to depth speckle.
    k = max(1, int(round(top_k_frac * roi.size)))
    nearest = np.sort(roi)[-k:]
    p_coll = float(np.clip(nearest.mean() / intensity_max, 0.0, 1.0))

    # Delta_r: forward clearance proxy. The depth that produced the closest
    # pixel is ~ intensity_max / peak_intensity. Normalize by r_max -> [0,1].
    peak = float(nearest.max())
    clearance_m = intensity_max / max(peak, 1.0)
    delta_r = float(np.clip(clearance_m / r_max, 0.0, 1.0))

    return p_coll, delta_r
