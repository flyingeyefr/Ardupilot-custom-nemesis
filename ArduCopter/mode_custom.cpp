#include "Copter.h"

/*
  Mode CUSTOM (29)

  Entrée : CH10 en position haute (RC10_OPTION = 318) ou failsafe radio (FS_THR_ENABLE = 8).
  Vol : freinage horizontal, montée de 30 m, coupure des moteurs 60 s après l'entrée.
  Sortie : radio valide et CH10 hors position haute, retour en LOITER,
           ou en STABILIZE si l'estimation de position est insuffisante.

  Règle à respecter : dans la boucle où le mode demande un changement de mode,
  les contrôleurs doivent tourner normalement avant l'appel à set_mode().
  AC_PosControl considère le contrôleur vertical actif s'il a tourné à la boucle
  précédente (D_is_active). Si une boucle est sautée, LOITER le reprend sans
  l'initialiser et D_update_controller() lève INTERNAL_ERROR(flow_of_control)
  à la ligne 1082 de AC_PosControl.cpp. L'erreur reste mémorisée, seul un
  redémarrage l'efface, et elle bloque l'armement. Le changement de mode se
  fait donc en fin de run(), comme dans ModeBrake::run().
*/

bool ModeCustom::init(bool ignore_checks)
{
    if (copter.position_ok()) {
        pos_control->NE_set_max_speed_accel_m(pos_control->get_vel_estimate_NED_ms().xy().length(), BRAKE_MODE_DECEL_RATE_MSS);
        pos_control->NE_set_correction_speed_accel_m(pos_control->get_vel_estimate_NED_ms().xy().length(), BRAKE_MODE_DECEL_RATE_MSS);

        if (!pos_control->NE_is_active()) {
            pos_control->NE_init_controller();
        }
    }

    pos_control->D_set_max_speed_accel_m(BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_DECEL_RATE_MSS);
    pos_control->D_set_correction_speed_accel_m(BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_DECEL_RATE_MSS);

    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    fs_start_time_ms = AP_HAL::millis();
    last_exit_attempt_ms = 0;

    target_alt_m = pos_control->get_pos_estimate_U_m() + 30.0f;

    return true;
}

void ModeCustom::run()
{
    // coupure des moteurs 60 s après l'entrée dans le mode
    if (motors->armed() && (AP_HAL::millis() - fs_start_time_ms >= 60000)) {
        copter.arming.disarm(AP_Arming::Method::TERMINATION);
    }

    if (!motors->armed()) {
        // au sol ou après la coupure : contrôleurs relâchés et mis à jour à chaque boucle
        make_safe_ground_handling();
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        float target_roll_cd = 0.0f;
        float target_pitch_cd = 0.0f;

        // freinage puis maintien horizontal si la position est fiable
        if (copter.position_ok()) {
            if (!pos_control->NE_is_active()) {
                pos_control->NE_init_controller();
            }
            Vector2f target_vel_xy(0.0f, 0.0f);
            Vector2f target_accel_xy(0.0f, 0.0f);
            pos_control->input_vel_accel_NE_cm(target_vel_xy, target_accel_xy);
            pos_control->NE_update_controller();

            target_roll_cd = pos_control->get_roll_cd();
            target_pitch_cd = pos_control->get_pitch_cd();
        }

        // montée vers target_alt_m puis maintien à +/- 1 m
        const float current_alt_m = pos_control->get_pos_estimate_U_m();
        float target_climb_speed_cms = 0.0f;

        if (current_alt_m < (target_alt_m - 1.0f)) {
            target_climb_speed_cms = 150.0f;
        } else if (current_alt_m > (target_alt_m + 1.0f)) {
            target_climb_speed_cms = -80.0f;
        }

        pos_control->D_set_pos_target_from_climb_rate_cms(target_climb_speed_cms);
        pos_control->D_update_controller();

        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll_cd, target_pitch_cd, 0.0f);
    }

    // changement de mode en dernier, après la mise à jour des contrôleurs de cette boucle
    if (exit_requested()) {
        exit_to_pilot_mode();
    }
}

// vrai si la radio est valide et si l'interrupteur CUSTOM_MODE n'est pas en position haute
bool ModeCustom::exit_requested() const
{
    // aucune sortie pendant le failsafe radio
    if (!rc().has_valid_input()) {
        return false;
    }

    // canal configuré avec RCx_OPTION = 318 (CUSTOM_MODE), le canal 10 sur Nemesis
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CUSTOM_MODE);
    if (chan == nullptr) {
        // aucun interrupteur configuré : retour au pilote dès le retour radio
        return true;
    }

    // position haute au-dessus de 1800 µs, seuils et inversion identiques à la fonction auxiliaire
    return chan->get_aux_switch_pos() != RC_Channel::AuxSwitchPos::HIGH;
}

// retour au pilote : LOITER si la position est fiable, sinon STABILIZE
void ModeCustom::exit_to_pilot_mode()
{
    // une tentative par seconde au maximum si les deux modes sont refusés,
    // pour éviter une rafale de messages d'échec à 400 Hz
    const uint32_t now_ms = AP_HAL::millis();
    if ((last_exit_attempt_ms != 0) && (now_ms - last_exit_attempt_ms < 1000)) {
        return;
    }
    last_exit_attempt_ms = now_ms;

    if (copter.position_ok() && copter.set_mode(Mode::Number::LOITER, ModeReason::RADIO_FAILSAFE_RECOVERY)) {
        return;
    }
    copter.set_mode(Mode::Number::STABILIZE, ModeReason::RADIO_FAILSAFE_RECOVERY);
}
