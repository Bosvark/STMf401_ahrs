#ifndef TYPES_H_
#define TYPES_H_

#define ACC
#define BARO
#define VARIOMETER
#define BUZZER
#define INFLIGHT_ACC_CALIBRATION

enum box {
	BOXARM,
#ifdef ACC
	BOXANGLE,
	BOXHORIZON,
#endif
#if defined( BARO ) && (!defined( SUPPRESS_BARO_ALTHOLD ))
	BOXBARO,
#endif
#ifdef VARIOMETER
	BOXVARIO,
#endif
	BOXMAG,
#if defined(HEADFREE)
	BOXHEADFREE,
	BOXHEADADJ, // acquire heading for HEADFREE mode
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
	BOXCAMSTAB,
#endif
#if defined(CAMTRIG)
	BOXCAMTRIG,
#endif
#if GPS
	BOXGPSHOME,
	BOXGPSHOLD,
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
	BOXPASSTHRU,
#endif
#if defined(BUZZER)
	BOXBEEPERON,
#endif
#if defined(LED_FLASHER)
	BOXLEDMAX, // we want maximum illumination
	BOXLEDLOW, // low/no lights
#endif
#if defined(LANDING_LIGHTS_DDR)
	BOXLLIGHTS, // enable landing lights at any altitude
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
	BOXCALIB,
#endif
#ifdef GOVERNOR_P
	BOXGOV,
#endif
#ifdef OSD_SWITCH
	BOXOSD,
#endif
#if GPS
	BOXGPSNAV,
	BOXLAND,
#endif
	CHECKBOXITEMS
};




#endif /* TYPES_H_ */
