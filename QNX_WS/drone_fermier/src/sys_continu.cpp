/*
 * sys_continu.cpp
 *
 *  Created on: 21 oct. 2020
 *      Author: dalusa
 */
#include <pthread.h>
#include <math.h>
#include <cstdio>
#include "sys_continu.h"
#include "output_csv.h"
#include "drone.h"
#include <errno.h>
#include <float.h>

#define PI 3.14159265

//tâches d'interface au code existant (tâches longues)
void interface_prendrePhoto(SysContinu* sys)
{
    auto startTime = std::chrono::steady_clock::now();

	//on transforme la base de (0,0) au pixel correspondant dans le bitmap
	coord_t photoCoord = sys->posxy.load();
	photoCoord.x += 273;
	photoCoord.y += 204;
    sys->cameraModule.takePhoto(photoCoord);
    sys->prendrePhotoEnCours.store(false);//on indique que l'opération est terminée
    sys->photo_prise.store(true);

    auto endTime = std::chrono::steady_clock::now();
    double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime)).count();
    sys->bgp_lastRuntime.store(elapsed);
    if (elapsed > sys->bgp_worstRuntime.load())
    {
        sys->bgp_worstRuntime.store(elapsed);
    }
}

void interface_transmettrePhoto(SysContinu* sys)
{
    auto startTime = std::chrono::steady_clock::now();

    sys->cameraModule.transmitPhotos();//on ingnore le ID retourné
    sys->transmissionEnCours.store(false);
    sys->photo_trans_handler();

    auto endTime = std::chrono::steady_clock::now();
    double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime)).count();
    sys->bgt_lastRuntime.store(elapsed);
    if (elapsed > sys->bgt_worstRuntime.load())
    {
        sys->bgt_worstRuntime.store(elapsed);
    }
}

//tâches périodiques
void print_cont(SysContinu* sys)
{
    auto slp_until = std::chrono::steady_clock::now();
    message_t msg;
    while (true)
    {
        slp_until += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(slp_until);//on print au maximum un message au 20 ms

        //print un messages dans la queue
        if (!sys->msgQId.empty())
        {
            msg = sys->msgQId.pop();
            printf(msg.log);
            printf("\n");
        }

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        sys->print_lastRuntime.store(elapsed);
        if (elapsed > sys->print_worstRuntime.load())
        {
            sys->print_worstRuntime.store(elapsed);
        }
        if (elapsed > 20.0)
        {
            sys->print_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
}

void logState_cont(SysContinu* sys)
{
    auto slp_until = std::chrono::steady_clock::now();
    message_t msg;
    unsigned long long loopCounter;
    //init csv file
    csvInit(sys->csvFile);
    while (true)
    {
        slp_until += std::chrono::milliseconds(100);
        std::this_thread::sleep_until(slp_until);

        outputLine(sys->csvFile, (loopCounter*100), sys, get_gdata());

        if (loopCounter%10 == 0)//every second
        {
			//log l'état du système (print)
			float v[7];
			coord_t coord = sys->posxy.load();
			v[0] = coord.x;
			v[1] = coord.y;
			v[2] = sys->posz.load();
			v[3] = sys->orientation.load();
			v[4] = sys->vitesse_h.load();
			v[5] = sys->vitesse_v.load();
			v[6] = sys->charge_batterie.load();

			snprintf(&(msg.log[0]), MAX_MSG_SIZE, "Position: (%f, %f, %f). Orientation %f. Vitesse (%f, %f). Batterie %f/100.",
					 v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
			sys->msgQId.push(msg);
        }

        loopCounter++;

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        sys->log_lastRuntime.store(elapsed);
        if (elapsed > sys->log_worstRuntime.load())
        {
            sys->log_worstRuntime.store(elapsed);
        }
        if (elapsed > 100.0)
        {
            sys->log_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
}


void camera_cont(SysContinu* sys)
{
    auto slp_until = std::chrono::steady_clock::now();

    while (true)
    {
        slp_until += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(slp_until);

        if (sys->in_prendrePhoto.load())//commande de prendre photo
        {
            if (sys->prendrePhotoEnCours.load())
            {
                message_t msg;
                snprintf(&(msg.log[0]), MAX_MSG_SIZE, "Prendre photo reçu alors que la dernière photo est encore en progrès.");
                sys->msgQId.push(msg);
                sys->in_prendrePhoto.store(false);
                continue;
            }
            sys->in_prendrePhoto.store(false);
            sys->prendrePhotoEnCours.store(true);

            //on part le thread de prise de photo qui va updater l'output une fois fini
            struct sched_param mySchedParam;
            sys->t_takePhoto_bg = std::thread(&interface_prendrePhoto, sys);
            mySchedParam.sched_priority = 5;
            pthread_setschedparam(sys->t_takePhoto_bg.native_handle(), SCHED_FIFO, &mySchedParam);
            sys->t_takePhoto_bg.detach();
        }

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        sys->cam_lastRuntime.store(elapsed);
        if (elapsed > sys->cam_worstRuntime.load())
        {
            sys->cam_worstRuntime.store(elapsed);
        }
        if (elapsed > 20.0)
        {
            sys->cam_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
}

void orientation_cont(SysContinu* sys)
{
    auto slp_until = std::chrono::steady_clock::now();

    while (true)
    {
        slp_until += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(slp_until);

        sys->orientation.store(sys->in_orientation.load());

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        sys->or_lastRuntime.store(elapsed);
        if (elapsed > sys->or_worstRuntime.load())
        {
            sys->or_worstRuntime.store(elapsed);
        }
        if (elapsed > 20.0)
        {
            sys->or_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
}

void vitesse_cont(SysContinu* sys)
{
    auto slp_until = std::chrono::steady_clock::now();

    while (true)
    {
        slp_until += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(slp_until);

        //On assume une constante de temps de 50ms
        vitesse_t past_spd = sys->vxy.load();
        double past_spd_v = sys->vitesse_v.load();
        double angle = 90 - sys->orientation.load();//angle normal xy au lieu de boussole
        vitesse_t new_spd_unfiltered;//vitesse de polaire à cartesien
        double v_h = sys->in_vitesse_h.load();
        double v_v = sys->in_vitesse_v.load();
        new_spd_unfiltered.x = v_h*cos(angle * PI/180);
        new_spd_unfiltered.y = v_h*sin(angle * PI/180);
        vitesse_t new_spd;//on rajoute une constante de temps à la vitesse
        double past_coeff = 0.8187/SIMU_ACCEL;
        new_spd.x = (1-past_coeff)*new_spd_unfiltered.x + past_coeff*past_spd.x;
        new_spd.y = (1-past_coeff)*new_spd_unfiltered.y + past_coeff*past_spd.y;
        sys->vxy.store(new_spd);
        //transforme en polaire pour output
        sys->vitesse_h.store( sqrt(pow(new_spd.x,2) + pow(new_spd.y,2)) );
        double nv = (1-past_coeff)*v_v + past_coeff*past_spd_v;
        sys->vitesse_v.store(nv);

        //mise à jour de la position
        double old_posz = sys->posz.load();
        coord_t old_posxy = sys->posxy.load();
        coord_t nposxy;
        nposxy.x = old_posxy.x + SIMU_ACCEL*new_spd.x/50.0;
        nposxy.y = old_posxy.y + SIMU_ACCEL*new_spd.y/50.0;
        sys->posxy.store(nposxy);
        sys->posz.store(old_posz + SIMU_ACCEL*nv/50.0);//50 periodes de 20ms par secondes, vitesse en m/s

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        sys->vit_lastRuntime.store(elapsed);
        if (elapsed > sys->vit_worstRuntime.load())
        {
            sys->vit_worstRuntime.store(elapsed);
        }
        if (elapsed > 20.0)
        {
            sys->vit_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
}

void transmission_cont(SysContinu* sys)
{
    auto slp_until = std::chrono::steady_clock::now();

    while (true)
    {
        slp_until += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(slp_until);

        if (sys->in_transmission.load())//commande de transmettre photo
        {
            if (sys->transmissionEnCours.load())
            {
                message_t msg;
                snprintf(&(msg.log[0]), MAX_MSG_SIZE, "Transmettre photo reçu alors que la dernière transmission est encore en progrès.");
                sys->msgQId.push(msg);
                sys->in_transmission.store(false);
                continue;
            }
            sys->in_transmission.store(false);
            sys->transmissionEnCours.store(true);

            //on part le thread de prise de photo qui va updater l'output une fois fini
            struct sched_param mySchedParam;
            sys->t_transmitPhoto_bg = std::thread(&interface_transmettrePhoto, sys);
            mySchedParam.sched_priority = 5;
            pthread_setschedparam(sys->t_transmitPhoto_bg.native_handle(), SCHED_FIFO, &mySchedParam);
            sys->t_transmitPhoto_bg.detach();
        }

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        sys->trans_lastRuntime.store(elapsed);
        if (elapsed > sys->trans_worstRuntime.load())
        {
            sys->trans_worstRuntime.store(elapsed);
        }
        if (elapsed > 20.0)
        {
            sys->trans_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
}

void batterie_cont(SysContinu* sys)
{
    auto slp_until = std::chrono::steady_clock::now();

    while (true)
    {
        slp_until += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(slp_until);

        bool enCharge = sys->batterieEnCharge.load();

        if (enCharge)
        {
            //on assume un temps de recharge de 10 minutes -> 600s
            //donc on charge 1% par 6 secondes, 1/600 % par tranche de 10ms
            double charge = sys->charge_batterie.load();
            charge += (8.0/600.0) * SIMU_ACCEL;
            if (charge > 99.9)
            {
                charge = 100;
            }
            sys->charge_batterie.store(charge);
            if (charge > 99.9)
            {
            	sys->batterieFaibleFired.store(false);//reset
                sys->batterie_100_handler();
                sys->batterieEnCharge.store(false);
            }
        }
        else
        {
            //On assume pour faire simple que la batterie est proportionnelle à la vitesse horizontale + une constante par temps.
            double charge = sys->charge_batterie.load();
            charge -= ((14.0/3600.0) + (4.0/3600)*sys->vitesse_h.load())*SIMU_ACCEL;
            if (charge <= 10)
            {
                if (charge < 0)
                {
                    charge = 0;
                    message_t msg;
                    snprintf(&(msg.log[0]), MAX_MSG_SIZE, "La charge de la batterie est descendu sous zero.");
                    sys->msgQId.push(msg);
                    sys->in_transmission.store(false);
                }
            }
            sys->charge_batterie.store(charge);
            if (charge < 10 && (sys->batterieFaibleFired.load() == false))
            {
                sys->batterie_10_handler();
                sys->batterieFaibleFired.store(true);
            }
        }

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        sys->bat_lastRuntime.store(elapsed);
        if (elapsed > sys->bat_worstRuntime.load())
        {
            sys->bat_worstRuntime.store(elapsed);
        }
        if (elapsed > 20.0)
        {
            sys->bat_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
}


SysContinu::SysContinu(void (*handler_batterie_10)(void), void (*handler_batterie_100)(void),
    void (*handler_photo_transmise)(void))
	:msgQId(MAX_NUM_MESSAGES), csvFile("simulationResult.csv", std::ofstream::trunc)
    {
        this->batterie_10_handler = handler_batterie_10;
        this->batterie_100_handler = handler_batterie_100;
        this->photo_trans_handler = handler_photo_transmise;
        //init les attributs
        this->charge_batterie.store(100);
        this->photo_prise.store(false);
        this->posxy.store(coord_t({0,0}));
        this->posz.store(0);
        this->orientation.store(0);
        this->vitesse_h.store(0);
        this->vitesse_v.store(0);
        this->batterieEnCharge.store(false);
        this->transmissionEnCours.store(false);
        this->prendrePhotoEnCours.store(false);
        this->vxy.store(vitesse_t({0,0}));
        this->in_transmission.store(false);
        this->in_prendrePhoto.store(false);
        this->in_orientation.store(0);
        this->in_vitesse_h.store(0);
        this->in_vitesse_v.store(0);
        this->batterieFaibleFired.store(false);
        //métriques de thread
        this->cam_worstRuntime.store(0);
        this->cam_lastRuntime.store(0);
        this->cam_dlnMissed.store(0);
        this->or_worstRuntime.store(0);
        this->or_lastRuntime.store(0);
        this->or_dlnMissed.store(0);
        this->vit_worstRuntime.store(0);
        this->vit_lastRuntime.store(0);
        this->vit_dlnMissed.store(0);
        this->trans_worstRuntime.store(0);
        this->trans_lastRuntime.store(0);
        this->trans_dlnMissed.store(0);
        this->bat_worstRuntime.store(0);
        this->bat_lastRuntime.store(0);
        this->bat_dlnMissed.store(0);
        this->log_worstRuntime.store(0);
        this->log_lastRuntime.store(0);
        this->log_dlnMissed.store(0);
        this->print_worstRuntime.store(0);
        this->print_lastRuntime.store(0);
        this->print_dlnMissed.store(0);
        this->bgp_worstRuntime.store(0);
        this->bgp_lastRuntime.store(0);
        this->bgt_worstRuntime.store(0);
        this->bgt_lastRuntime.store(0);
        //crééer les 5 tâches périodiques (camera, orientation, vitesse, transmission, batterie) + une print (1 every seconde/SIMU_ACCEL)
        struct sched_param mySchedParam;

        this->t_camera_cont = std::thread(&camera_cont, this);
        mySchedParam.sched_priority = 20;
        pthread_setschedparam(this->t_camera_cont.native_handle(), SCHED_FIFO, &mySchedParam);

        this->t_orientation_cont = std::thread(&orientation_cont, this);
        mySchedParam.sched_priority = 20;
        pthread_setschedparam(this->t_orientation_cont.native_handle(), SCHED_FIFO, &mySchedParam);

        this->t_vitesse_cont = std::thread(&vitesse_cont, this);
        mySchedParam.sched_priority = 20;
        pthread_setschedparam(this->t_vitesse_cont.native_handle(), SCHED_FIFO, &mySchedParam);

        this->t_transmission_cont = std::thread(&transmission_cont, this);
        mySchedParam.sched_priority = 20;
        pthread_setschedparam(this->t_transmission_cont.native_handle(), SCHED_FIFO, &mySchedParam);

        this->t_batterie_cont = std::thread(&batterie_cont, this);
        mySchedParam.sched_priority = 20;
        pthread_setschedparam(this->t_batterie_cont.native_handle(), SCHED_FIFO, &mySchedParam);

        this->t_logState_cont = std::thread(&logState_cont, this);
        mySchedParam.sched_priority = 20;
        pthread_setschedparam(this->t_logState_cont.native_handle(), SCHED_FIFO, &mySchedParam);

        this->t_print_cont = std::thread(&print_cont, this);
        mySchedParam.sched_priority = 25;
        pthread_setschedparam(this->t_print_cont.native_handle(), SCHED_FIFO, &mySchedParam);
    }

//fonctions de l'API
void SysContinu::set_charger_batterie()
{
    this->batterieEnCharge.store(true);
}

void SysContinu::set_transmettre_photo()
{
    this->in_transmission.store(true);
}

void SysContinu::set_prendre_photo()
{
    this->photo_prise.store(false);
    this->in_prendrePhoto.store(true);
}

void SysContinu::set_consigne_orientation(double x)
{
    this->in_orientation.store(x);
}

void SysContinu::set_vitesse_h(double x)
{
    this->in_vitesse_h.store(x);
}

void SysContinu::set_vitesse_v(double x)
{
    this->in_vitesse_v.store(x);
}

double SysContinu::get_niv_batterie()
{
    return this->charge_batterie.load();
}

double SysContinu::get_photo_prise()
{
    return this->photo_prise.load();
}

coord_t SysContinu::get_posxy()
{
    return this->posxy.load();
}

double SysContinu::get_posz()
{
    return this->posz.load();
}

double SysContinu::get_orientation()
{
    return this->orientation.load();
}

double SysContinu::get_vitesse_h()
{
    return this->vitesse_h.load();
}

double SysContinu::get_vitesse_v()
{
    return this->vitesse_v.load();
}
