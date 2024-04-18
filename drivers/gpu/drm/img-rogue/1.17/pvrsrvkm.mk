pvrsrvkm_1_17-y += \
 client_cache_direct_bridge.o \
 server_cache_bridge.o \
 server_cmm_bridge.o \
 client_devicememhistory_direct_bridge.o \
 server_devicememhistory_bridge.o \
 server_di_bridge.o \
 server_dmabuf_bridge.o \
 client_htbuffer_direct_bridge.o \
 server_htbuffer_bridge.o \
 client_mm_direct_bridge.o \
 server_mm_bridge.o \
 client_pvrtl_direct_bridge.o \
 server_pvrtl_bridge.o \
 server_rgxbreakpoint_bridge.o \
 server_rgxcmp_bridge.o \
 server_rgxfwdbg_bridge.o \
 server_rgxhwperf_bridge.o \
 server_rgxkicksync_bridge.o \
 server_rgxregconfig_bridge.o \
 server_rgxta3d_bridge.o \
 server_rgxtimerquery_bridge.o \
 server_rgxtq2_bridge.o \
 server_rgxtq_bridge.o \
 server_srvcore_bridge.o \
 client_sync_direct_bridge.o \
 server_sync_bridge.o \
 client_synctracking_direct_bridge.o \
 server_synctracking_bridge.o \
 cache_km.o \
 connection_server.o \
 debug_common.o \
 devicemem_heapcfg.o \
 devicemem_history_server.o \
 devicemem_server.o \
 di_impl_brg.o \
 di_server.o \
 handle.o \
 htb_debug.o \
 htbserver.o \
 info_page_km.o \
 lists.o \
 mmu_common.o \
 physheap.o \
 physmem.o \
 physmem_hostmem.o \
 physmem_lma.o \
 pmr.o \
 power.o \
 process_stats.o \
 pvr_notifier.o \
 pvrsrv.o \
 pvrsrv_bridge_init.o \
 pvrsrv_pool.o \
 srvcore.o \
 sync_checkpoint.o \
 sync_server.o \
 tlintern.o \
 tlserver.o \
 tlstream.o \
 vmm_pvz_client.o \
 vmm_pvz_server.o \
 vz_vmm_pvz.o \
 vz_vmm_vm.o \
 rgx_bridge_init.o \
 rgxbreakpoint.o \
 rgxbvnc.o \
 rgxccb.o \
 rgxfwdbg.o \
 rgxfwimageutils.o \
 rgxfwtrace_strings.o \
 rgxhwperf_common.o \
 rgxkicksync.o \
 rgxmem.o \
 rgxregconfig.o \
 rgxshader.o \
 rgxsyncutils.o \
 rgxtimecorr.o \
 rgxtimerquery.o \
 rgxutils.o \
 rgxcompute.o \
 rgxdebug.o \
 rgxfwutils.o \
 rgxhwperf.o \
 rgxinit.o \
 rgxlayer_impl.o \
 rgxmipsmmuinit.o \
 rgxmmuinit.o \
 rgxmulticore.o \
 rgxpower.o \
 rgxsrvinit.o \
 rgxstartstop.o \
 rgxta3d.o \
 rgxtdmtransfer.o \
 rgxtransfer.o \
 allocmem.o \
 event.o \
 fwload.o \
 handle_idr.o \
 km_apphint.o \
 module_common.o \
 osconnection_server.o \
 osfunc.o \
 osmmap_stub.o \
 physmem_dmabuf.o \
 physmem_osmem_linux.o \
 pmr_os.o \
 pvr_bridge_k.o \
 pvr_buffer_sync.o \
 pvr_debug.o \
 pvr_debugfs.o \
 pvr_drm.o \
 pvr_dvfs_device.o \
 pvr_fence.o \
 pvr_gputrace.o \
 pvr_platform_drv.o \
 pvr_counting_timeline.o \
 pvr_counting_timeline.o \
 pvr_sw_fence.o \
 pvr_sw_fence.o \
 pvr_sync_file.o \
 pvr_sync_ioctl_common.o \
 pvr_sync_ioctl_drm.o \
 devicemem.o \
 devicemem_utils.o \
 hash.o \
 htbuffer.o \
 mem_utils.o \
 pvrsrv_error.o \
 ra.o \
 sync.o \
 tlclient.o \
 uniq_key_splay_tree.o \
 rgx_hwperf_table.o \
 interrupt_support.o \
 dma_support.o \
 vmm_type_stub.o
pvrsrvkm_1_17-$(CONFIG_DRM_POWERVR_ROGUE_DEBUG) += \
 client_ri_direct_bridge.o \
 server_ri_bridge.o \
 ri_server.o
pvrsrvkm_1_17-$(CONFIG_DRM_POWERVR_ROGUE_PDUMP) += \
 client_pdump_direct_bridge.o \
 server_pdump_bridge.o \
 client_pdumpctrl_direct_bridge.o \
 server_pdumpctrl_bridge.o \
 client_pdumpmm_direct_bridge.o \
 server_pdumpmm_bridge.o \
 client_rgxpdump_direct_bridge.o \
 server_rgxpdump_bridge.o \
 pdump_mmu.o \
 pdump_physmem.o \
 pdump_server.o \
 rgxpdump.o \
 devicemem_pdump.o \
 devicememx_pdump.o
ifneq ($(CONFIG_DRM_POWERVR_ROGUE_PDUMP),y)
pvrsrvkm_1_17-y += \
 physmem_test.o
endif
pvrsrvkm_1_17-$(CONFIG_ARM)   += osfunc_arm.o
pvrsrvkm_1_17-$(CONFIG_ARM64) += osfunc_arm64.o
pvrsrvkm_1_17-$(CONFIG_EVENT_TRACING) += trace_events.o
pvrsrvkm_1_17-$(CONFIG_RISCV) += osfunc_riscv.c
pvrsrvkm_1_17-$(CONFIG_X86)   += osfunc_x86.o
