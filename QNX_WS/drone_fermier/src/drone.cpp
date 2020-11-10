#include <iostream>
#include <atomic>
#include <time.h>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>
#include <sys/types.h>
#include <sys/syspage.h>
#include <sys/neutrino.h>
#include "sys_continu.h"
#include "drone.h"
#include <float.h>

#define PAS_DE_SIMULATION 10E-3
#define PI 3.141592
#define CTRL_CAMERA_DISTANCE_PHOTO	5.0
#define CTRL_CAMERA_PERIOD_MS       100
#define CTRL_NAVIGATION_PERIOD_MS  	100
#define CTRL_CAMERA_PRIORITY 		11
#define CTRL_NAVIGATION_PRIORITY	10
#define STATION_BATTERIE coord_t({0,0})

//coordonnées pour scanner le champs 22
#define CHAMPS_22_COORDS coord_t({12.5,12.5}),coord_t({57.5,12.5}),coord_t({57.5,17.5}),coord_t({12.5,17.5}),coord_t({12.5,22.5}),coord_t({57.5,22.5}),coord_t({57.5,27.5}),coord_t({12.5,27.5}),coord_t({12.5,32.5}),coord_t({57.5,32.5}),\
                         coord_t({57.5,37.5}),coord_t({12.5,37.5}),coord_t({12.5,42.5}),coord_t({57.5,42.5}),coord_t({57.5,47.5}),coord_t({43.5,47.5}),coord_t({43.5,77.5}),coord_t({38.5,77.5}),coord_t({38.5,47.5}),coord_t({33.5,47.5}),\
                         coord_t({33.5,77.5}),coord_t({28.5,77.5}),coord_t({28.5,47.5}),coord_t({23.5,47.5}),coord_t({23.5,77.5}),coord_t({20.5,77.5}),coord_t({20.5,47.5}),coord_t({12.5,47.5})

using namespace std;

TaskData *g_data;
SysContinu* g_sys;

pthread_t pthread_ctrl_camera_id;
pthread_t pthread_ctrl_navigation_id;

struct TaskData* get_gdata()
{
	return g_data;
}

void handler_batterie_10()
{
	cout << "Batterie faible" << endl;
    coord_t pos = g_sys->get_posxy();
    //sauvegarde les données de la mission précédente
    g_data->p_ctrlMission->cameraSaved_cycleActif.store(g_data->p_ctrlCamera->cycleActif.load());
    g_data->p_ctrlMission->navSaved_paused.store(g_data->p_ctrlNavigation->paused.load());
    g_data->p_ctrlMission->navSaved_destinationAtteinteAction.store(g_data->p_ctrlNavigation->destinationAtteinteAction.load());
    g_data->p_ctrlMission->navSaved_destination.store(g_data->p_ctrlNavigation->destination.load());
    g_data->p_ctrlMission->saved_position.store(pos);
    //update destination et planifie la recharge de la batterie
    g_data->p_ctrlNavigation->destination.store(STATION_BATTERIE);
    g_data->p_ctrlCamera->cycleActif.store(false);
    g_data->p_ctrlNavigation->destinationAtteinteAction.store(act_chargeBatterie);
}
void handler_batterie_100()
{
	cout << "Batterie rechargée" << endl;
    //retour à la dernière position du drone
    g_data->p_ctrlNavigation->destination.store(g_data->p_ctrlMission->saved_position.load());
    //planifie de reprendre l'état sauvegardé lorsque la destination est atteinte
    g_data->p_ctrlNavigation->destinationAtteinteAction.store(act_rechargeMission);
}
void handler_photo_transmise()
{
    //vide memoire photo (sans memset)
    g_data->p_ctrlCamera->memoireOffset.store(0);
    //redémarre navigation
    g_data->p_ctrlNavigation->paused.store(false);
    //redémarre le cycle de photos si il reste des photos à prendre
    int action = g_data->p_ctrlNavigation->destinationAtteinteAction.load();
    if (g_data->p_ctrlCamera->champsFini.load() == false && (action != act_chargeBatterie) && (action != act_rechargeMission))
    {
    	g_data->p_ctrlCamera->cycleActif.store(true);
    }
}

float calculer_distance (coord_t p1, coord_t p2)
{
	return (float)sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
}

//tâche du controleurCamera
void* ctrl_cam_function(void* arg) {
	cout << "Start camera controler" << endl;
    auto slp_until = std::chrono::steady_clock::now();

    while (true)
    {
        slp_until += std::chrono::milliseconds(CTRL_CAMERA_PERIOD_MS);
        std::this_thread::sleep_until(slp_until);

        coord_t pos_drone = g_sys->get_posxy();

        if (g_data->p_ctrlCamera->cycleActif.load() || g_data->p_ctrlCamera->takelastpicture.load())
        {
        	if (g_data->p_ctrlCamera->takelastpicture.load() && (g_data->p_ctrlCamera->photoWait.load() == false))
        	{
				g_data->p_ctrlCamera->photoWait.store(true);
				cout << "Photo prise (final)" << endl;
        		g_sys->set_prendre_photo();
				g_data->p_ctrlCamera->position_derniere_photo.store(pos_drone);
				time(&(g_data->p_ctrlCamera->timestamp));

				g_data->p_ctrlCamera->takelastpicture.store(false);

                cout << "Transmission photo" << endl;
                //on arrête les déplacements
                g_data->p_ctrlNavigation->paused.store(true);
                g_data->p_ctrlCamera->cycleActif.store(false);
                //on démarrre la transmission
                g_sys->set_transmettre_photo();
                continue;
        	}

            coord_t destination = g_data->p_ctrlNavigation->destination.load();
            bool destAtteinte = (g_data->p_ctrlNavigation->gps_lecture.x >= destination.x - 0.10)&&(g_data->p_ctrlNavigation->gps_lecture.x <= destination.x + 0.10)&&
                    (g_data->p_ctrlNavigation->gps_lecture.y >= destination.y - 0.10)&&(g_data->p_ctrlNavigation->gps_lecture.y <= destination.y + 0.10);

            bool prendPhotoDest = ((destAtteinte && g_data->p_ctrlCamera->photoDest.load() == false) && (g_data->p_ctrlCamera->photoWait.load() == false));

            //On vérifie si c'est le temps de prendre une nouvelle photo
            if (prendPhotoDest || calculer_distance(g_data->p_ctrlCamera->position_derniere_photo.load(), pos_drone) >= CTRL_CAMERA_DISTANCE_PHOTO)
            {
                g_data->p_ctrlCamera->photoWait.store(true);
                cout << "Photo prise" << endl;
                g_sys->set_prendre_photo();
                g_data->p_ctrlCamera->position_derniere_photo.store(pos_drone);
                time(&(g_data->p_ctrlCamera->timestamp));

                if (prendPhotoDest)
                {
                	g_data->p_ctrlCamera->photoDest.store(true);
                }
            }

            //Si la photo est reçu on la store en mémoire
            time_t t;
            time(&t);
            if (g_data->p_ctrlCamera->photoWait.load() && difftime(t,g_data->p_ctrlCamera->timestamp) < 4.8/SIMU_ACCEL)
            {
            	//photo non recue et pas encore en timeout
                if (g_sys->get_photo_prise())
                {
                    g_data->p_ctrlCamera->photoWait.store(false);
                    g_data->p_ctrlCamera->memoireOffset.fetch_add(20*1000000);
                    double memUsed = g_data->p_ctrlCamera->memoireOffset.load();

                    cout << "Memoire libre: " << g_data->p_ctrlCamera->tailleMemoire - memUsed << endl;

                    if (g_data->p_ctrlCamera->tailleMemoire - memUsed < 20*1000000*1.2)
                    {
                    	//memoire pleine
                        cout << "Transmission photo" << endl;
                        //on arrête les déplacements
                        g_data->p_ctrlNavigation->paused.store(true);
                        g_data->p_ctrlCamera->cycleActif.store(false);
                        //on démarrre la transmission
                        g_sys->set_transmettre_photo();
                    }
                }
            }
			else if (g_data->p_ctrlCamera->photoWait.load())
			{
				g_data->p_ctrlCamera->photoWait.store(false);
                g_data->p_ctrlCamera->photoManquee.fetch_add(1);
				cout << "Photo non recue" << endl;
			}
        }

        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        g_data->p_ctrlCamera->cc_lastRuntime.store(elapsed);
        if (elapsed > g_data->p_ctrlCamera->cc_worstRuntime.load())
        {
            g_data->p_ctrlCamera->cc_worstRuntime.store(elapsed);
        }
        if (elapsed > CTRL_CAMERA_PERIOD_MS)
        {
            g_data->p_ctrlCamera->cc_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
    return NULL;
}

//tâche du controleurNavigation
void* ctrl_nav_function(void* arg) {
	cout << "Start navigation controler" << endl;
    auto slp_until = std::chrono::steady_clock::now();

    while (true)
    {
        slp_until += std::chrono::milliseconds(CTRL_NAVIGATION_PERIOD_MS);
        std::this_thread::sleep_until(slp_until);
        //on vérifie si on est en mode pause déplacement
        if (g_data->p_ctrlNavigation->paused.load()){
            g_sys->set_vitesse_h(0.0);
	        g_sys->set_vitesse_v(0.0);
	        continue;
        }
        //On verifie si le drone est en mode photo oui-> faible vitesse et faible altitude
        if (g_data->p_ctrlCamera->cycleActif.load()){
            g_data->p_ctrlNavigation->cible_altitude.store(5.0);
            g_data->p_ctrlNavigation->cible_vitesse.store(1.0);
        }
        else{
            g_data->p_ctrlNavigation->cible_altitude.store(15.0);
            g_data->p_ctrlNavigation->cible_vitesse.store(10.0); 
        }
        //on va chercher la position et vitesse actuelle
        coord_t posxy = g_sys->get_posxy();
        g_data->p_ctrlNavigation->gps_lecture.x = posxy.x;
        g_data->p_ctrlNavigation->gps_lecture.y = posxy.y;
        g_data->p_ctrlNavigation->gps_lecture.z = g_sys->get_posz();
        g_data->p_ctrlNavigation->gps_lecture.vh = g_sys->get_vitesse_h();
        g_data->p_ctrlNavigation->gps_lecture.vv = g_sys->get_vitesse_v();

        //handler pour destination atteinte
        coord_t destination = g_data->p_ctrlNavigation->destination.load();
        bool destAtteinte = (g_data->p_ctrlNavigation->gps_lecture.x >= destination.x - 0.10)&&(g_data->p_ctrlNavigation->gps_lecture.x <= destination.x + 0.10)&&
                (g_data->p_ctrlNavigation->gps_lecture.y >= destination.y - 0.10)&&(g_data->p_ctrlNavigation->gps_lecture.y <= destination.y + 0.10);

        if ((destAtteinte && g_data->p_ctrlCamera->cycleActif.load() && g_data->p_ctrlCamera->photoDest.load() == false) || g_data->p_ctrlCamera->takelastpicture.load())
        {
        	//on attend que la photo soit prise
            g_sys->set_vitesse_h(0.0);
	        g_sys->set_vitesse_v(0.0);
	        continue;
        }

        if (destAtteinte)
		{
        	g_data->p_ctrlCamera->photoDest.store(false); //On change la destination, on n'a pas de photo de la future destination
			//x et y à l'intérieur de 10cm: destination atteinte
			coord_t firstpoint = g_data->p_ctrlNavigation->champs_actif.coords[0];
			coord_t lastpoint = g_data->p_ctrlNavigation->champs_actif.coords[g_data->p_ctrlNavigation->champs_actif.nbCoord - 1];
			if (destination.x == firstpoint.x && destination.y == firstpoint.y){
				//premier point du champs actif: on active le mode photo
				g_data->p_ctrlCamera->cycleActif.store(true);
			}
			else if (destination.x == lastpoint.x && destination.y == lastpoint.y && g_data->p_ctrlCamera->champsFini.load() == false){
				//dernier point du champs actif: on désactive le mode photo
				g_data->p_ctrlCamera->champsFini.store(true);
				g_data->p_ctrlCamera->cycleActif.store(false);
				g_data->p_ctrlCamera->takelastpicture.store(true);
				continue;
			}
			//handler pour batterie et recharge
			switch (g_data->p_ctrlNavigation->destinationAtteinteAction.load()) {
				case act_chargeBatterie:
					g_sys->set_vitesse_h(0.0);
					g_sys->set_vitesse_v(0.0);
					g_sys->set_charger_batterie();
					continue;//Pas besoin de choisir une nouvelle destination, le handler de trig de batterie pleine s'en occupe
				case act_rechargeMission:
					//on recharge la mission précédente
					g_data->p_ctrlCamera->cycleActif.store(g_data->p_ctrlMission->cameraSaved_cycleActif.load());
					g_data->p_ctrlNavigation->paused.store(g_data->p_ctrlMission->navSaved_paused.load());
					g_data->p_ctrlNavigation->destinationAtteinteAction.store(g_data->p_ctrlMission->navSaved_destinationAtteinteAction.load());
					g_data->p_ctrlNavigation->destination.store(g_data->p_ctrlMission->navSaved_destination.load());
					continue;//Pas besoin de choisir une nouvelle destination, on vient de le faire
				default:
					break;
			}
			//charge la prochaine destination: aucune = retour à la base. Base: base
			//on vérifie si la destination était une coordonnée du champs
			int coord_idx = -1;
			for (int i=0; i<g_data->p_ctrlNavigation->champs_actif.nbCoord; i++){
				if (g_data->p_ctrlNavigation->champs_actif.coords[i].x == destination.x && g_data->p_ctrlNavigation->champs_actif.coords[i].y == destination.y){
					coord_idx = i;
					break;
				}
			}
			//dernier point ou hors du champs
			if (coord_idx == (g_data->p_ctrlNavigation->champs_actif.nbCoord - 1) || coord_idx == -1){
				//On ne sait plus où aller, autant revenir/rester à la base.
				g_data->p_ctrlNavigation->destination.store(STATION_BATTERIE);
			}
			//sinon on prend la prochaine coordonnée
			else{
				g_data->p_ctrlNavigation->destination.store(g_data->p_ctrlNavigation->champs_actif.coords[coord_idx+1]);
			}
		}
        //on met à jour les consignes de vitesse et orientation
        //si destination encore atteinte on met la vitesse horizontale à 0
        destination = g_data->p_ctrlNavigation->destination.load();
        if ( (g_data->p_ctrlNavigation->gps_lecture.x >= destination.x - 0.10)&&(g_data->p_ctrlNavigation->gps_lecture.x <= destination.x + 0.10)&&
                (g_data->p_ctrlNavigation->gps_lecture.y >= destination.y - 0.10)&&(g_data->p_ctrlNavigation->gps_lecture.y <= destination.y + 0.10)){
            //x et y à l'intérieur de 10cm: destination atteinte
            g_sys->set_vitesse_h(0.0);
        }
        else if ( (g_data->p_ctrlNavigation->gps_lecture.x >= destination.x - 1)&&(g_data->p_ctrlNavigation->gps_lecture.x <= destination.x + 1)&&
                (g_data->p_ctrlNavigation->gps_lecture.y >= destination.y - 1)&&(g_data->p_ctrlNavigation->gps_lecture.y <= destination.y + 1)){
            //x et y à l'intérieur de 1m: vitesse réduite pour ne pas passer tout droit
            g_sys->set_vitesse_h(0.5);
        }
        else{
            g_sys->set_vitesse_h(g_data->p_ctrlNavigation->cible_vitesse.load());
        }
        //vitesse verticale. On vise un ajustement progressif de la vitesse, avec une consigne proportielle à l'erreur de position, k=2
        g_sys->set_vitesse_v(2*(g_data->p_ctrlNavigation->cible_altitude.load() - g_data->p_ctrlNavigation->gps_lecture.z));
        //orientation
        double dx = destination.x - g_data->p_ctrlNavigation->gps_lecture.x;
        double dy = destination.y - g_data->p_ctrlNavigation->gps_lecture.y;
        double orient = atan2(dy, dx) * 180 / PI;
        //conversion en degrés nord-sud
        orient = 90 - orient;
        g_sys->set_consigne_orientation(orient);


        auto endTime = std::chrono::steady_clock::now();
        double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - slp_until)).count();
        g_data->p_ctrlNavigation->cn_lastRuntime.store(elapsed);
        if (elapsed > g_data->p_ctrlNavigation->cn_worstRuntime.load())
        {
            g_data->p_ctrlNavigation->cn_worstRuntime.store(elapsed);
        }
        if (elapsed > CTRL_NAVIGATION_PERIOD_MS)
        {
            g_data->p_ctrlNavigation->cn_dlnMissed.fetch_add(1);
            slp_until = endTime;
        }
    }
	return NULL;
}

//fonction init du noyau
void init(){
	cout << "Start init simulation" << endl << endl;
    g_data = new struct TaskData();
    memset( g_data, 0, sizeof(TaskData) );
    //init  les valeurs de cameraControleur
    g_data->p_ctrlCamera = new struct ControleurCamera;
    g_data->p_ctrlCamera->cycleActif.store(false);
    g_data->p_ctrlCamera->tailleMemoire = 512*1000000;//512MB
    g_data->p_ctrlCamera->memoireOffset = 0;
    g_data->p_ctrlCamera->photoDest.store(false);
    g_data->p_ctrlCamera->champsFini.store(false);
    g_data->p_ctrlCamera->takelastpicture.store(false);
    g_data->p_ctrlCamera->photoWait.store(false);
    g_data->p_ctrlCamera->photoManquee.store(0);
    g_data->p_ctrlCamera->cc_worstRuntime.store(0);
    g_data->p_ctrlCamera->cc_lastRuntime.store(0);
    g_data->p_ctrlCamera->cc_dlnMissed.store(0);
    //init vlaeurs controleurNavigation
    g_data->p_ctrlNavigation = new struct ControleurNavigation;
    g_data->p_ctrlNavigation->paused.store(false);
    g_data->p_ctrlNavigation->destinationAtteinteAction.store(act_changeDestination);
    g_data->p_ctrlNavigation->champs_actif.nbCoord = 28;
    g_data->p_ctrlNavigation->champs_actif.coords = new coord_t[28]{CHAMPS_22_COORDS};
    g_data->p_ctrlNavigation->cn_worstRuntime.store(0);
    g_data->p_ctrlNavigation->cn_lastRuntime.store(0);
    g_data->p_ctrlNavigation->cn_dlnMissed.store(0);
    //initie le drône pour scanner le premier champs
    g_data->p_ctrlNavigation->destination.store(g_data->p_ctrlNavigation->champs_actif.coords[0]);
    //init valeurs controleurMission
    g_data->p_ctrlMission = new struct ControleurMission;
    //les valeurs du struct controleurMission n'ont pas besoin d'être initialisé vu que l'on est assuré qu'ils seront écrits avant d'être lu par la machine à état du controleur.

    g_sys =  new SysContinu(handler_batterie_10, handler_batterie_100, handler_photo_transmise);

    pthread_attr_t pthread_attr_ctrl_camera;
    pthread_attr_t pthread_attr_ctrl_navigation;

    struct sched_param sched_ctrl_camera;
    struct sched_param sched_ctrl_navigation;

    pthread_attr_init(&pthread_attr_ctrl_camera);
    pthread_attr_init(&pthread_attr_ctrl_navigation);

    // Schedule
	sched_ctrl_camera.sched_priority = CTRL_CAMERA_PRIORITY;
	sched_ctrl_navigation.sched_priority = CTRL_NAVIGATION_PRIORITY;

	pthread_attr_setschedparam (&pthread_attr_ctrl_camera, &sched_ctrl_camera);
	pthread_attr_setschedparam (&pthread_attr_ctrl_navigation, &sched_ctrl_navigation);

    // Creation des thread
    cout << "Thread of camera controler" << endl;
    pthread_create(&pthread_ctrl_camera_id, &pthread_attr_ctrl_camera, &ctrl_cam_function, NULL);
    cout << "Thread of camera controler created" << endl;
    cout << "Thread of navigation controler" << endl;
	pthread_create(&pthread_ctrl_navigation_id, &pthread_attr_ctrl_navigation, &ctrl_nav_function, NULL);
	cout << "Thread of navigation controler created" << endl;

    cout << "Init simulation ended" << endl << endl;
}

//fonction de nettoyage du noyau
void cleanup() {
    // Free the allocated memory
	cout << "Stop all thread" << endl;

	pthread_detach ( pthread_ctrl_camera_id);
	pthread_detach ( pthread_ctrl_navigation_id);
	pthread_cancel ( pthread_ctrl_camera_id);
	pthread_cancel ( pthread_ctrl_navigation_id);

    delete g_sys;
    delete[] g_data->p_ctrlNavigation->champs_actif.coords;
    delete g_data->p_ctrlNavigation;
    delete g_data->p_ctrlMission;
    delete g_data->p_ctrlCamera;
    delete g_data;
    g_sys = NULL;
    g_data = NULL;

    cout << "Simulation ended" << endl;
}

int main() {
	init();

	time_t t, temps_debut;
	time(&t);
	time(&temps_debut);
	while (difftime(t,temps_debut) < 1200)
	{
		sleep(1);
		time(&t);
	}
	// Fin de simulation
	cleanup();
	return 0;
}
