/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_COLOR_H__
#define __INTEL_COLOR_H__

#include <linux/types.h>

struct intel_crtc_state;
struct intel_crtc;
struct drm_i915_private;
struct drm_property_blob;

void intel_color_init_hooks(struct drm_i915_private *i915);
int intel_color_init(struct drm_i915_private *i915);
void intel_color_crtc_init(struct intel_crtc *crtc);
int intel_color_check(struct intel_crtc_state *crtc_state);
void intel_color_commit_noarm(const struct intel_crtc_state *crtc_state);
void intel_color_commit_arm(const struct intel_crtc_state *crtc_state);
void intel_color_post_update(const struct intel_crtc_state *crtc_state);
void intel_color_load_luts(const struct intel_crtc_state *crtc_state);
void intel_color_get_config(struct intel_crtc_state *crtc_state);
int intel_color_get_gamma_bit_precision(const struct intel_crtc_state *crtc_state);
bool intel_color_lut_equal(struct drm_property_blob *blob1,
			   struct drm_property_blob *blob2,
			   u32 gamma_mode, u32 bit_precision);
void intel_color_assert_luts(const struct intel_crtc_state *crtc_state);

#endif /* __INTEL_COLOR_H__ */
