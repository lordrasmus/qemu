
/* raise an exception, exiting the cpu loop */
DEF_HELPER_2(raise_exception, void, env, i32)

#if !defined(CONFIG_USER_ONLY)

/* block thread due to a MetaBlock blocker */
DEF_HELPER_2(block, void, env, i32)
/* update IRQEnc with highest priority trigger */
DEF_HELPER_1(update_irqenc, void, env)
/* enable thread */
DEF_HELPER_1(thread_enable, void, env)
/* put the thread in the off state (but not stopped) */
DEF_HELPER_1(thread_disable, void, env)
/* TXENABLE.ThreadEnable = 0 to stop the thread */
DEF_HELPER_1(thread_stop, void, env)
/* Trigger a halt from target code. */
DEF_HELPER_1(halt, void, env)
/* Trigger a halt for a specific signum. */
DEF_HELPER_2(halt_signum, noreturn, env, i32)
/* Trigger and return 1 if interrupts outstanding */
DEF_HELPER_1(unmask_itrigger, i32, env)
/* replay catch buffers. */
DEF_HELPER_1(replay_catch, void, env)
/* modify lock state to allow another thread to take the lock */
DEF_HELPER_3(unlock, i32, env, i32, i32)

#endif
/* generate a deferred trigger */
DEF_HELPER_2(defr_trigger, void, env, i32)
#if !defined(CONFIG_USER_ONLY)
/* read TXSTAT/TXPOLL potentially with deferred triggers */
DEF_HELPER_FLAGS_1(get_txpoll, TCG_CALL_NO_RWG, i32, env)
/* read TXSTATI/TXPOLLI potentially with deferred triggers */
DEF_HELPER_FLAGS_1(get_txpolli, TCG_CALL_NO_RWG, i32, env)
/* external read TXSTAT/TXPOLL potentially with deferred triggers */
DEF_HELPER_FLAGS_1(txpoll_read, TCG_CALL_NO_RWG, i32, env)
/* external read TXSTATI/TXPOLLI potentially with deferred triggers */
DEF_HELPER_FLAGS_1(txpolli_read, TCG_CALL_NO_RWG, i32, env)
/* DEFR from TXPOLL */
DEF_HELPER_FLAGS_1(defr_txpoll, TCG_CALL_NO_RWG, i32, env)
/* DEFR from TXPOLLI */
DEF_HELPER_FLAGS_1(defr_txpolli, TCG_CALL_NO_RWG, i32, env)
/* DEFR from TXSTAT (acks) */
DEF_HELPER_1(defr_txstat, i32, env)
/* DEFR from TXSTATI (acks) */
DEF_HELPER_1(defr_txstati, i32, env)
/* ack highest priority deferred background trigger bits */
DEF_HELPER_2(ack_defr_txstat, void, env, i32)
/* ack highest priority deferred interrupt trigger bits */
DEF_HELPER_2(ack_defr_txstati, void, env, i32)
/* external ack highest priority deferred background trigger bits */
DEF_HELPER_2(txstat_ack_defr, void, env, i32)
/* external ack highest priority deferred interrupt trigger bits */
DEF_HELPER_2(txstati_ack_defr, void, env, i32)

/* trace events */
DEF_HELPER_2(ttevent0, void, i32, i32)
DEF_HELPER_3(ttevent1, void, i32, i32, i32)
DEF_HELPER_4(ttevent2, void, i32, i32, i32, i32)
DEF_HELPER_5(ttevent3, void, i32, i32, i32, i32, i32)
DEF_HELPER_6(ttevent4, void, i32, i32, i32, i32, i32, i32)

DEF_HELPER_FLAGS_4(cachew, TCG_CALL_NO_WG, void, env, i32, i32, i32)

/* trace events */
DEF_HELPER_3(ifetch, void, env, i32, i32)
DEF_HELPER_5(dload, void, env, i32, i32, i32, i32)
DEF_HELPER_5(dstore, void, env, i32, i32, i32, i32)
DEF_HELPER_4(dprefetch, void, env, i32, i32, i32)

#endif /* CONFIG_USER_ONLY */

DEF_HELPER_FLAGS_1(ffb, TCG_CALL_NO_RWG, s32, s32)
DEF_HELPER_FLAGS_1(ffb16, TCG_CALL_NO_RWG, s32, s32)
DEF_HELPER_FLAGS_1(norm, TCG_CALL_NO_RWG, s32, s32)
DEF_HELPER_FLAGS_1(norm16, TCG_CALL_NO_RWG, s32, s32)
DEF_HELPER_FLAGS_2(cacherl,  TCG_CALL_NO_RWG, i64, env, i32)

DEF_HELPER_FLAGS_3(fx_add32, TCG_CALL_NO_RWG, i32, env, i32, i32)
DEF_HELPER_FLAGS_3(fx_add64, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fx_mul32, TCG_CALL_NO_RWG, i32, env, i32, i32)
DEF_HELPER_FLAGS_3(fx_mul64, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fx_sub32, TCG_CALL_NO_RWG, i32, env, i32, i32)
DEF_HELPER_FLAGS_3(fx_sub64, TCG_CALL_NO_RWG, i64, env, i64, i64)

DEF_HELPER_FLAGS_5(fx_muz32, TCG_CALL_NO_RWG, f32, env, f32, f32, f32, int)
DEF_HELPER_FLAGS_5(fx_muz64, TCG_CALL_NO_RWG, f64, env, f64, f64, f64, int)

DEF_HELPER_FLAGS_3(fx_rcp32, TCG_CALL_NO_RWG, f32, env, f32, int)
DEF_HELPER_FLAGS_3(fx_rcp64, TCG_CALL_NO_RWG, f64, env, f64, int)
DEF_HELPER_FLAGS_3(fx_rsq32, TCG_CALL_NO_RWG, f32, env, f32, int)
DEF_HELPER_FLAGS_3(fx_rsq64, TCG_CALL_NO_RWG, f64, env, f64, int)

DEF_HELPER_FLAGS_2(fx_ftod, TCG_CALL_NO_RWG, f64, env, f32)
DEF_HELPER_FLAGS_2(fx_dtof, TCG_CALL_NO_RWG, f32, env, f64)
DEF_HELPER_FLAGS_2(fx_ftoh, TCG_CALL_NO_RWG, i32, env, f32)
DEF_HELPER_FLAGS_2(fx_dtoh, TCG_CALL_NO_RWG, i32, env, f64)
DEF_HELPER_FLAGS_2(fx_htof, TCG_CALL_NO_RWG, f32, env, i32)
DEF_HELPER_FLAGS_2(fx_htod, TCG_CALL_NO_RWG, f64, env, i32)
DEF_HELPER_FLAGS_3(fx_ftoi, TCG_CALL_NO_RWG, s32, env, f32, int)
DEF_HELPER_FLAGS_3(fx_dtoi, TCG_CALL_NO_RWG, s32, env, f64, int)
DEF_HELPER_FLAGS_2(fx_itof, TCG_CALL_NO_RWG, f32, env, s32)
DEF_HELPER_FLAGS_2(fx_itod, TCG_CALL_NO_RWG, f64, env, s32)
DEF_HELPER_FLAGS_3(fx_dtol, TCG_CALL_NO_RWG, s64, env, f64, int)
DEF_HELPER_FLAGS_2(fx_ltod, TCG_CALL_NO_RWG, f64, env, s64)
DEF_HELPER_FLAGS_3(fx_ftox, TCG_CALL_NO_RWG, s32, env, f32, int)
DEF_HELPER_FLAGS_3(fx_dtox, TCG_CALL_NO_RWG, s32, env, f64, int)
DEF_HELPER_FLAGS_3(fx_xtof, TCG_CALL_NO_RWG, f32, env, s32, int)
DEF_HELPER_FLAGS_3(fx_xtod, TCG_CALL_NO_RWG, f64, env, s32, int)
DEF_HELPER_FLAGS_3(fx_dtoxl, TCG_CALL_NO_RWG, s64, env, f64, int)
DEF_HELPER_FLAGS_3(fx_xltod, TCG_CALL_NO_RWG, f64, env, s64, int)

DEF_HELPER_FLAGS_4(fx_cmp32, TCG_CALL_NO_WG, f32, env, f32, f32, int)
DEF_HELPER_FLAGS_4(fx_cmp64, TCG_CALL_NO_WG, f64, env, f64, f64, int)

DEF_HELPER_3(fx_handle_exception, void, env, i32, i32)

DEF_HELPER_4(dspram_radix, i64, i32, i32, i32, i32)
DEF_HELPER_FLAGS_3(dspram_read, 0, i32, env, int, i32)
DEF_HELPER_FLAGS_4(dspram_write, 0, void, env, int, i32, i32)
DEF_HELPER_FLAGS_6(dspram_ptr_r_set, 0, i32, env, int, int, int, i32, ptr)
DEF_HELPER_FLAGS_6(dspram_ptr_w_set, 0, void, env, int, int, int, i32, ptr)

DEF_HELPER_FLAGS_4(au_add, 0, i32, env, int, i32, i32)

DEF_HELPER_FLAGS_2(daoppame_invalidate_template, 0, void, env, i32)

DEF_HELPER_FLAGS_2(read_append_ra, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_rabz, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_rawz, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_radz, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_rabx, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_rawx, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_radx, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_ram8x, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_ram8x32, 0, void, env, i64)
DEF_HELPER_FLAGS_2(read_append_ram16x, 0, void, env, i64)

DEF_HELPER_FLAGS_1(sim_start, 0, void, env)
DEF_HELPER_FLAGS_1(sim_stop, 0, void, env)
DEF_HELPER_FLAGS_2(sim_dump, 0, void, env, i32)

DEF_HELPER_FLAGS_3(shift_rspp_sats9, TCG_CALL_NO_RWG_SE, i64, i64, i32, i64)
DEF_HELPER_FLAGS_3(shift_rspp_satu8, TCG_CALL_NO_RWG_SE, i64, i64, i32, i64)
DEF_HELPER_FLAGS_3(saturate_asl_i40, 0, i64, i64, i64, i32)
DEF_HELPER_FLAGS_1(mul_ps_sat16, TCG_CALL_NO_RWG_SE, i32, i32)
DEF_HELPER_FLAGS_3(add16_overflow, TCG_CALL_NO_RWG_SE, i32, i32, i32, i32)
DEF_HELPER_FLAGS_3(sub16_overflow, TCG_CALL_NO_RWG_SE, i32, i32, i32, i32)
DEF_HELPER_FLAGS_3(add16_sat, TCG_CALL_NO_RWG_SE, i32, i32, i32, i32)
DEF_HELPER_FLAGS_3(sub16_sat, TCG_CALL_NO_RWG_SE, i32, i32, i32, i32)
DEF_HELPER_FLAGS_1(addsub16_pshiftse, TCG_CALL_NO_RWG_SE, i32, i32)


/* register handlers, called through a host function pointer */
DEF_HELPER_2(reg_read, i32, env, ptr)
DEF_HELPER_2(reg_read64, i64, env, ptr)
DEF_HELPER_3(reg_write, void, env, ptr, i32)
DEF_HELPER_4(reg_writel, void, env, ptr, i32, i32)
