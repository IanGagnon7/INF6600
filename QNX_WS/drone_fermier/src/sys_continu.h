#ifndef __SYS_CONTINU_H__
#define __SYS_CONTINU_H__

#include <atomic>
#include <thread>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <sys/syspage.h>
#include <sys/neutrino.h>
#include "cameraModule.h"
#include "common_queue.h"

#define MAX_NUM_MESSAGES 50
#define MAX_MSG_SIZE 256

typedef struct
{
	int errorlevel;
	char log[MAX_MSG_SIZE];
} message_t;

typedef struct
{
    double x;
    double y;
} vitesse_t;

class SysContinu
{
public:
	/*Les handlers de batterie doivent être < 5ms pour ne pas perturber l'opérations des tâches.*/
	SysContinu(void (*handler_batterie_10)(void), void (*handler_batterie_100)(void),
			void (*handler_photo_transmise)(void));

	//public API
	void set_charger_batterie();
	void set_transmettre_photo();
	void set_prendre_photo();
	void set_consigne_orientation(double x);
	void set_vitesse_h(double x);
	void set_vitesse_v(double x);

	double get_niv_batterie();
	double get_photo_prise();
	coord_t get_posxy();
	double get_posz();
	double get_orientation();
	double get_vitesse_h();
	double get_vitesse_v();

	//file de messages
	nsCommon::Queue<message_t> msgQId;

	//variables d'état locales à sys_continu.cpp
	//inputs
	std::atomic<bool> in_transmission;
	std::atomic<bool> in_prendrePhoto;
	std::atomic<double> in_orientation;
	std::atomic<double> in_vitesse_h;
	std::atomic<double> in_vitesse_v;
	//outputs
	std::atomic<double> charge_batterie;
	std::atomic<bool> photo_prise;
	std::atomic<coord_t> posxy;
	std::atomic<double> posz;
	std::atomic<double> orientation;
	std::atomic<double> vitesse_h;
	std::atomic<double> vitesse_v;

	//internal state
	std::atomic<bool> batterieEnCharge;
	std::atomic<bool> transmissionEnCours;
	std::atomic<bool> prendrePhotoEnCours;
	std::atomic<bool> batterieFaibleFired;
	std::atomic<vitesse_t> vxy;

	PathMap cameraModule;

	std::ofstream csvFile;

	void (*batterie_10_handler)(void);
	void (*batterie_100_handler)(void);
	void (*photo_trans_handler)(void);

	//threads
	std::thread t_camera_cont;
	std::atomic<double> cam_worstRuntime;
	std::atomic<double> cam_lastRuntime;
	std::atomic<unsigned int> cam_dlnMissed;
	std::thread t_orientation_cont;
	std::atomic<double> or_worstRuntime;
	std::atomic<double> or_lastRuntime;
	std::atomic<unsigned int> or_dlnMissed;
	std::thread t_vitesse_cont;
	std::atomic<double> vit_worstRuntime;
	std::atomic<double> vit_lastRuntime;
	std::atomic<unsigned int> vit_dlnMissed;
	std::thread t_transmission_cont;
	std::atomic<double> trans_worstRuntime;
	std::atomic<double> trans_lastRuntime;
	std::atomic<unsigned int> trans_dlnMissed;
	std::thread t_batterie_cont;
	std::atomic<double> bat_worstRuntime;
	std::atomic<double> bat_lastRuntime;
	std::atomic<unsigned int> bat_dlnMissed;

	std::thread t_logState_cont;
	std::atomic<double> log_worstRuntime;
	std::atomic<double> log_lastRuntime;
	std::atomic<unsigned int> log_dlnMissed;
	std::thread t_print_cont;
	std::atomic<double> print_worstRuntime;
	std::atomic<double> print_lastRuntime;
	std::atomic<unsigned int> print_dlnMissed;

	std::thread t_takePhoto_bg;
	std::atomic<double> bgp_worstRuntime;
	std::atomic<double> bgp_lastRuntime;
	std::thread t_transmitPhoto_bg;
	std::atomic<double> bgt_worstRuntime;
	std::atomic<double> bgt_lastRuntime;
};

#endif
