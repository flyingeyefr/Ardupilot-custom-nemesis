#include "Copter.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */

bool Copter::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

void Copter::failsafe_radio_on_event()
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    // set desired action based on FS_THR_ENABLE parameter
    FailsafeAction desired_action;
    switch (g.failsafe_throttle) {
        case FS_THR_DISABLED:
            desired_action = FailsafeAction::NONE;
            break;
        case FS_THR_ENABLED_ALWAYS_RTL:
        case FS_THR_ENABLED_CONTINUE_MISSION:
            desired_action = FailsafeAction::RTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_THR_ENABLED_ALWAYS_LAND:
            desired_action = FailsafeAction::LAND;
            break;
        case FS_THR_ENABLED_AUTO_RTL_OR_RTL:
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_THR_ENABLED_BRAKE_OR_LAND:
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        case FS_THR_ENABLED_STABILIZE_CLIMB_DISARM:
            //handled below, LAND is the fallback if Stabilize cannot be entered
            desired_action = FailsafeAction::LAND;
            break;
        default:
            desired_action = FailsafeAction::LAND;
    }

    // Conditions to deviate from FS_THR_ENABLE selection and send specific GCS warning
    if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        announce_failsafe("Radio", "Disarming");
        arming.disarm(AP_Arming::Method::RADIOFAILSAFE);
        desired_action = FailsafeAction::NONE;

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Allow landing to continue when battery failsafe requires it (not a user option)
        announce_failsafe("Radio + Battery", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // Allow landing to continue when FS_OPTIONS is set to continue landing
        announce_failsafe("Radio", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->mode_number() == Mode::Number::AUTO && failsafe_option(FailsafeOption::RC_CONTINUE_IF_AUTO)) {
        // Allow mission to continue when FS_OPTIONS is set to continue mission
        announce_failsafe("Radio", "Continuing Auto");
        desired_action = FailsafeAction::NONE;

    } else if ((flightmode->in_guided_mode()) && failsafe_option(FailsafeOption::RC_CONTINUE_IF_GUIDED)) {
        // Allow guided mode to continue when FS_OPTIONS is set to continue in guided mode
        announce_failsafe("Radio", "Continuing Guided Mode");
        desired_action = FailsafeAction::NONE;

    } else if (g.failsafe_throttle == FS_THR_ENABLED_STABILIZE_CLIMB_DISARM) {
        // Stabilize, climb at hover + FS_THR_CLB_THR for FS_THR_CLB_TIME, then disarm.
        //If the video-loss procedure is already running, it continues with its original timer
        if (lost_link_start(ModeReason::RADIO_FAILSAFE)) {
            announce_failsafe("Radio", "Stabilize climb");
            desired_action = FailsafeAction::NONE;
        } else {
            // desired_action stays LAND (set in the switch above)
            announce_failsafe("Radio", "Stabilize failed, Land");
        }

    } else {
        announce_failsafe("Radio");
    }

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

// failsafe_off_event - respond to radio contact being regained
void Copter::failsafe_radio_off_event()
{
    // the lost-link procedure is reset in lost_link_update(), once the video-loss switch is clear too

    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

// true while the pilot's video-loss switch (FS_VID_CH) is below FS_VID_PWM
bool Copter::video_loss_switch_active() const
{
    const int8_t ch = g2.fs_vid_ch.get();
    if (ch <= 0 || ch > NUM_RC_CHANNELS) {
        return false;
    }
    const RC_Channel *c = rc().channel(ch - 1);
    if (c == nullptr) {
        return false;
    }
    const int16_t pwm = c->get_radio_in();
    // below 800 means no data on that channel, never treat it as "video lost"
    return (pwm >= 800) && (pwm < g2.fs_vid_pwm.get());
}

// mode-change reasons that may interrupt the lost-link climb, other failsafes only
bool Copter::lost_link_allows_mode_change(ModeReason reason) const
{
    switch (reason) {
    case ModeReason::BATTERY_FAILSAFE:
    case ModeReason::GCS_FAILSAFE:
    case ModeReason::EKF_FAILSAFE:
    case ModeReason::FENCE_BREACHED:
    case ModeReason::TERRAIN_FAILSAFE:
    case ModeReason::CRASH_FAILSAFE:
    case ModeReason::DEADRECKON_FAILSAFE:
    case ModeReason::AVOIDANCE:
    case ModeReason::TERMINATE:
        return true;
    default:
        // RC mode switch, aux switches, GCS commands, scripting...
        return false;
    }
}

// true while the lost-link climb is running (used by Stabilize)
bool Copter::lost_link_climb_active() const
{
    return (lost_link.state == LostLinkState::ACTIVE) && motors->armed();
}

// enter Stabilize and start the lost-link climb, keeps the original timer if already running
bool Copter::lost_link_start(ModeReason reason)
{
    if (lost_link.state == LostLinkState::ACTIVE && flightmode->mode_number() == Mode::Number::STABILIZE) {
        return true;
    }
    if (!set_mode(Mode::Number::STABILIZE, reason)) {
        return false;
    }
    lost_link.state = LostLinkState::ACTIVE;
    lost_link.start_ms = millis();
    // the pilot throttle stick is ignored from now on: don't let a low stick
    // count as "pilot wants motors off" (failsafe disarm, fence disarm, ground idle)
    ap.throttle_zero = false;
    return true;
}

// called from rc_loop: starts the procedure on video-loss switch, resets it when all
// links are back, aborts it on mode change and disarms after FS_THR_CLB_TIME
void Copter::lost_link_update()
{
    const bool rc_lost = failsafe.radio && (g.failsafe_throttle == FS_THR_ENABLED_STABILIZE_CLIMB_DISARM);
    const bool vid_lost = video_loss_switch_active();

    switch (lost_link.state) {
    case LostLinkState::IDLE:
        // RC loss is started from failsafe_radio_on_event(), here only the video switch
        if (vid_lost && motors->armed() && !ap.land_complete) {
            if (lost_link_start(ModeReason::FAILSAFE)) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Video lost - Stabilize climb");
            } else {
                // don't retry every loop, wait for the switch to be released
                lost_link.state = LostLinkState::ABORTED;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Video lost - Stabilize failed");
            }
        }
        break;

    case LostLinkState::ACTIVE: {
        if (!motors->armed()) {
            lost_link.state = LostLinkState::IDLE;
            break;
        }
        if (!rc_lost && !vid_lost) {
            lost_link.state = LostLinkState::IDLE;
            gcs().send_text(MAV_SEVERITY_WARNING, "Links restored - climb stopped");
            break;
        }
        if (flightmode->mode_number() != Mode::Number::STABILIZE) {
            // only another failsafe can get here (pilot/GCS mode changes are refused in set_mode)
            lost_link.state = LostLinkState::ABORTED;
            gcs().send_text(MAV_SEVERITY_WARNING, "Lost-link climb stopped by %s", flightmode->name());
            break;
        }
        const uint32_t climb_ms = uint32_t(MAX(g2.fs_thr_climb_time.get(), 0.0f) * 1000.0f);
        if (millis() - lost_link.start_ms >= climb_ms) {
            lost_link.state = LostLinkState::IDLE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Lost-link climb done - Disarming");
            arming.disarm(rc_lost ? AP_Arming::Method::RADIOFAILSAFE : AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
        }
        break;
    }

    case LostLinkState::ABORTED:
        // another failsafe took over (or Stabilize could not be entered)
        // re-arm only once RC is OK and the video-loss switch is clear
        if (!rc_lost && !vid_lost) {
            lost_link.state = LostLinkState::IDLE;
        }
        break;
    }
}


void Copter::announce_failsafe(const char *type, const char *action_undertaken)
{
    if (action_undertaken != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe - %s", type, action_undertaken);
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe", type);
    }
}

void Copter::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    FailsafeAction desired_action = (FailsafeAction)action;

    // Conditions to deviate from BATT_FS_XXX_ACT parameter setting
    if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
        desired_action = FailsafeAction::NONE;
        announce_failsafe("Battery", "Disarming");

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING) && desired_action != FailsafeAction::NONE) {
        // Allow landing to continue when FS_OPTIONS is set to continue when landing
        desired_action = FailsafeAction::LAND;
        announce_failsafe("Battery", "Continuing Landing");
    } else {
        announce_failsafe("Battery");
    }

    // Battery FS options already use the Failsafe_Options enum. So use them directly.
    do_failsafe_action(desired_action, ModeReason::BATTERY_FAILSAFE);

}

// failsafe_gcs_check - check for ground station failsafe
void Copter::failsafe_gcs_check()
{
    // Bypass GCS failsafe checks if disabled or GCS never connected
    if (g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    // Determine which event to trigger
    if (last_gcs_update_ms < gcs_timeout_ms && failsafe.gcs) {
        // Recovery from a GCS failsafe
        set_failsafe_gcs(false);
        failsafe_gcs_off_event();

    } else if (last_gcs_update_ms < gcs_timeout_ms && !failsafe.gcs) {
        // No problem, do nothing

    } else if (last_gcs_update_ms > gcs_timeout_ms && failsafe.gcs) {
        // Already in failsafe, do nothing

    } else if (last_gcs_update_ms > gcs_timeout_ms && !failsafe.gcs) {
        // New GCS failsafe event, trigger events
        set_failsafe_gcs(true);
        failsafe_gcs_on_event();
    }
}

// failsafe_gcs_on_event - actions to take when GCS contact is lost
void Copter::failsafe_gcs_on_event(void)
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_OCCURRED);
    RC_Channels::clear_overrides();

    // convert the desired failsafe response to the FailsafeAction enum
    FailsafeAction desired_action;
    switch (g.failsafe_gcs) {
        case FS_GCS_DISABLED:
            desired_action = FailsafeAction::NONE;
            break;
        case FS_GCS_ENABLED_ALWAYS_RTL:
        case FS_GCS_ENABLED_CONTINUE_MISSION:
            desired_action = FailsafeAction::RTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_GCS_ENABLED_ALWAYS_LAND:
            desired_action = FailsafeAction::LAND;
            break;
        case FS_GCS_ENABLED_AUTO_RTL_OR_RTL:
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_GCS_ENABLED_BRAKE_OR_LAND:
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        default: // if an invalid parameter value is set, the fallback is RTL
            desired_action = FailsafeAction::RTL;
    }

    // Conditions to deviate from FS_GCS_ENABLE parameter setting
    if (!motors->armed()) {
        desired_action = FailsafeAction::NONE;
        announce_failsafe("GCS");

    } else if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        arming.disarm(AP_Arming::Method::GCSFAILSAFE);
        desired_action = FailsafeAction::NONE;
        announce_failsafe("GCS", "Disarming");

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Allow landing to continue when battery failsafe requires it (not a user option)
        announce_failsafe("GCS + Battery", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // Allow landing to continue when FS_OPTIONS is set to continue landing
        announce_failsafe("GCS", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->mode_number() == Mode::Number::AUTO && failsafe_option(FailsafeOption::GCS_CONTINUE_IF_AUTO)) {
        // Allow mission to continue when FS_OPTIONS is set to continue mission
        announce_failsafe("GCS", "Continuing Auto Mode");
        desired_action = FailsafeAction::NONE;

    } else if (failsafe_option(FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL) && !flightmode->is_autopilot()) {
        // should continue when in a pilot controlled mode because FS_OPTIONS is set to continue in pilot controlled modes
        announce_failsafe("GCS", "Continuing Pilot Control");
        desired_action = FailsafeAction::NONE;
    } else {
        announce_failsafe("GCS");
    }

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::GCS_FAILSAFE);
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
void Copter::failsafe_gcs_off_event(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe Cleared");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_RESOLVED);
}

// executes terrain failsafe if data is missing for longer than a few seconds
void Copter::failsafe_terrain_check()
{
    // trigger within <n> milliseconds of failures while in various modes
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    bool trigger_event = timeout && flightmode->requires_terrain_failsafe();

    // check for clearing of event
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            failsafe_terrain_on_event();
        } else {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

// set terrain data status (found or not found)
void Copter::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = millis();

    // record time of first and latest failures (i.e. duration of failures)
    if (!data_ok) {
        failsafe.terrain_last_failure_ms = now;
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // failures cleared after 0.1 seconds of persistent successes
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}

// terrain failsafe action
void Copter::failsafe_terrain_on_event()
{
    failsafe.terrain = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Failsafe: Terrain data missing");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::FAILSAFE_OCCURRED);

    if (should_disarm_on_failsafe()) {
        arming.disarm(AP_Arming::Method::TERRAINFAILSAFE);
#if MODE_RTL_ENABLED
    } else if (flightmode->mode_number() == Mode::Number::RTL) {
        mode_rtl.restart_without_terrain();
#endif
    } else {
        set_mode_RTL_or_land_with_pause(ModeReason::TERRAIN_FAILSAFE);
    }
}

// check for gps glitch failsafe
void Copter::gpsglitch_check()
{
    // get filter status
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    bool gps_glitching = filt_status.flags.gps_glitching;

    // log start or stop of gps glitch.  AP_Notify update is handled from within AP_AHRS
    if (ap.gps_glitching != gps_glitching) {
        ap.gps_glitching = gps_glitching;
        if (gps_glitching) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch or Compass error");
        } else {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::ERROR_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Glitch cleared");
        }
    }
}

// dead reckoning alert and failsafe
void Copter::failsafe_deadreckon_check()
{
    // update dead reckoning state
    const char* dr_prefix_str = "Dead Reckoning";

    // get EKF filter status
    bool ekf_dead_reckoning = inertial_nav.get_filter_status().flags.dead_reckoning;

    // alert user to start or stop of dead reckoning
    const uint32_t now_ms = AP_HAL::millis();
    if (dead_reckoning.active != ekf_dead_reckoning) {
        dead_reckoning.active = ekf_dead_reckoning;
        if (dead_reckoning.active) {
            dead_reckoning.start_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s started", dr_prefix_str);
        } else {
            dead_reckoning.start_ms = 0;
            dead_reckoning.timeout = false;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s stopped", dr_prefix_str);
        }
    }

    // check for timeout
    if (dead_reckoning.active && !dead_reckoning.timeout) {
        const uint32_t dr_timeout_ms = uint32_t(constrain_float(g2.failsafe_dr_timeout * 1000.0f, 0.0f, UINT32_MAX));
        if (now_ms - dead_reckoning.start_ms > dr_timeout_ms) {
            dead_reckoning.timeout = true;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s timeout", dr_prefix_str);
        }
    }

    // exit immediately if deadreckon failsafe is disabled
    if (g2.failsafe_dr_enable <= 0) {
        failsafe.deadreckon = false;
        return;
    }

    // check for failsafe action
    if (failsafe.deadreckon != ekf_dead_reckoning) {
        failsafe.deadreckon = ekf_dead_reckoning;

        // only take action in modes requiring position estimate
        if (failsafe.deadreckon && copter.flightmode->requires_GPS()) {

            // log error
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_DEADRECKON, LogErrorCode::FAILSAFE_OCCURRED);

            // immediately disarm while landed
            if (should_disarm_on_failsafe()) {
                arming.disarm(AP_Arming::Method::DEADRECKON_FAILSAFE);
                return;
            }

            // take user specified action
            do_failsafe_action((FailsafeAction)g2.failsafe_dr_enable.get(), ModeReason::DEADRECKON_FAILSAFE);
        }
    }
}

// set_mode_RTL_or_land_with_pause - sets mode to RTL if possible or LAND with 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_RTL_or_land_with_pause(ModeReason reason)
{
#if MODE_RTL_ENABLED
    // attempt to switch to RTL, if this fails then switch to Land
    if (set_mode(Mode::Number::RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    // set mode to land will trigger mode change notification to pilot
    set_mode_land_with_pause(reason);
}

// set_mode_SmartRTL_or_land_with_pause - sets mode to SMART_RTL if possible or LAND with 4 second delay before descent starts
// this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_SmartRTL_or_land_with_pause(ModeReason reason)
{
#if MODE_SMARTRTL_ENABLED
    // attempt to switch to SMART_RTL, if this failed then switch to Land
    if (set_mode(Mode::Number::SMART_RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Using Land Mode");
    set_mode_land_with_pause(reason);
}

// set_mode_SmartRTL_or_RTL - sets mode to SMART_RTL if possible or RTL if possible or LAND with 4 second delay before descent starts
// this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_SmartRTL_or_RTL(ModeReason reason)
{
#if MODE_SMARTRTL_ENABLED
    // attempt to switch to SmartRTL, if this failed then attempt to RTL
    // if that fails, then land
    if (set_mode(Mode::Number::SMART_RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Trying RTL Mode");
    set_mode_RTL_or_land_with_pause(reason);
}

// Sets mode to Auto and jumps to DO_LAND_START, as set with AUTO_RTL param
// This can come from failsafe or RC option
void Copter::set_mode_auto_do_land_start_or_RTL(ModeReason reason)
{
#if MODE_AUTO_ENABLED
    if (set_mode(Mode::Number::AUTO_RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif

    gcs().send_text(MAV_SEVERITY_WARNING, "Trying RTL Mode");
    set_mode_RTL_or_land_with_pause(reason);
}

// Sets mode to Brake or LAND with 4 second delay before descent starts
// This can come from failsafe or RC option
void Copter::set_mode_brake_or_land_with_pause(ModeReason reason)
{
#if MODE_BRAKE_ENABLED
    if (set_mode(Mode::Number::BRAKE, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif

    gcs().send_text(MAV_SEVERITY_WARNING, "Trying Land Mode");
    set_mode_land_with_pause(reason);
}

bool Copter::should_disarm_on_failsafe() {
    if (ap.in_arming_delay) {
        return true;
    }

    switch (flightmode->mode_number()) {
        case Mode::Number::STABILIZE:
        case Mode::Number::ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero || ap.land_complete;
        case Mode::Number::AUTO:
        case Mode::Number::AUTO_RTL:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed && ap.land_complete;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            return ap.land_complete;
    }
}


void Copter::do_failsafe_action(FailsafeAction action, ModeReason reason){

    // Execute the specified desired_action
    switch (action) {
        case FailsafeAction::NONE:
            return;
        case FailsafeAction::LAND:
            set_mode_land_with_pause(reason);
            break;
        case FailsafeAction::RTL:
            set_mode_RTL_or_land_with_pause(reason);
            break;
        case FailsafeAction::SMARTRTL:
            set_mode_SmartRTL_or_RTL(reason);
            break;
        case FailsafeAction::SMARTRTL_LAND:
            set_mode_SmartRTL_or_land_with_pause(reason);
            break;
        case FailsafeAction::TERMINATE: {
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
            g2.afs.gcs_terminate(true, "Failsafe");
#else
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
#endif
            break;
        }
        case FailsafeAction::AUTO_DO_LAND_START:
            set_mode_auto_do_land_start_or_RTL(reason);
            break;
        case FailsafeAction::BRAKE_LAND:
            set_mode_brake_or_land_with_pause(reason);
            break;
    }

#if AP_GRIPPER_ENABLED
    if (failsafe_option(FailsafeOption::RELEASE_GRIPPER)) {
        gripper.release();
    }
#endif
}

