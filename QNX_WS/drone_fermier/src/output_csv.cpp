#include "output_csv.h"
#include <string>

using namespace std;

#define TCP_PACKET_SIZE 64 // La communication TCP utilise une taille de paquet fixe

const char * const to_string(bool b)
{
  return b ? "1" : "0";
}

void csvInit(ofstream &file)
{


    file << "pas de simulation (ms),charge batterie,position x,position y,position z,"
            "orientation,vitesse horizontale,vitesse verticale,transmission en cours,batterie en charge,"
            "prise de photo en cours,cycle camera actif,memoire photo libre,destination x,destination y,"
            "champs actif,cible vitesse,cible altitude,nb photo manquees,camera_cont last runtime,"
            "camera_cont worst runtime,camera_cont deadline manques,orientation_cont last runtime,orientation_cont worst runtime,orientation_cont deadline manques,"
            "vitesse_cont last runtime,vitesse_cont worst runtime,vitesse_cont deadline manques,transmission_cont last runtime,transmission_cont worst runtime,"
            "transmission_cont deadline manques,batterie_cont last runtime,batterie_cont worst runtime,batterie_cont deadline manques,logState_cont last runtime,"
            "logState_cont worst runtime,logState_cont deadline manques,print_cont last runtime,print_cont worst runtime,print_cont deadline manques,"
            "ctrl_camera last runtime,ctrl_camera worst runtime,ctrl_camera deadline manques,ctrl_navigation last runtime,ctrl_navigation worst runtime,"
            "ctrl_navigation deadline manques,takePhoto_bg last runtime,takePhoto_bg worst runtime,transmitPhoto_bg last runtime,transmitPhoto_bg worst runtime" << endl;
}

void outputLine(ofstream &csvFile, unsigned int msTime, SysContinu* sys, struct TaskData* tsk)
{
    coord_t posxy = sys->get_posxy();
    ControleurCamera* cc = tsk->p_ctrlCamera;
    ControleurNavigation* cn = tsk->p_ctrlNavigation;
    long long memoireLibre = cc->tailleMemoire - cc->memoireOffset.load();
    coord_t dest = cn->destination.load();

    // TRANSFERER LES DONNEES AU GUI (data = MEM, BATT, X, Y, Z)
    if (sys->get_socket() != -1){
		char* data_char_arr;
		string data_str = to_string(cc->memoireOffset.load())+","+to_string(sys->get_niv_batterie())+","+to_string(posxy.x)+","+to_string(posxy.y)+","+to_string(sys->get_posz());
		while (data_str.length() < TCP_PACKET_SIZE){
			data_str += " "; // Ajouter des espaces jusqu'a ce que le paquet soit de la bonne taille
		}
		data_char_arr = &data_str[0];
		send(sys->get_socket() , data_char_arr , TCP_PACKET_SIZE , 0 );
    }


    csvFile<<to_string(msTime)<<","<<to_string(sys->get_niv_batterie())<<","<<to_string(posxy.x)<<","<<to_string(posxy.y)<<","<<to_string(sys->get_posz())<<","<<
             to_string(sys->get_orientation())<<","<<to_string(sys->get_vitesse_h())<<","<<to_string(sys->get_vitesse_v())<<","<<to_string(sys->transmissionEnCours.load())<<","<<to_string(sys->batterieEnCharge.load())<<","<<
             to_string(sys->prendrePhotoEnCours.load())<<","<<to_string(cc->cycleActif.load())<<","<<to_string(memoireLibre)<<","<<to_string(dest.x)<<","<<to_string(dest.y)<<","<<
             "22"<<","<<to_string(cn->cible_vitesse.load())<<","<<to_string(cn->cible_altitude.load())<<","<<to_string(cc->photoManquee.load())<<","<<to_string(sys->cam_lastRuntime.load())<<","<<
             to_string(sys->cam_worstRuntime.load())<<","<<to_string(sys->cam_dlnMissed.load())<<","<<to_string(sys->or_lastRuntime.load())<<","<<to_string(sys->or_worstRuntime.load())<<","<<to_string(sys->or_dlnMissed.load())<<","<<
             to_string(sys->vit_lastRuntime.load())<<","<<to_string(sys->vit_worstRuntime.load())<<","<<to_string(sys->vit_dlnMissed.load())<<","<<to_string(sys->trans_lastRuntime.load())<<","<<to_string(sys->trans_worstRuntime.load())<<","<<
             to_string(sys->trans_dlnMissed.load())<<","<<to_string(sys->bat_lastRuntime.load())<<","<<to_string(sys->bat_worstRuntime.load())<<","<<to_string(sys->bat_dlnMissed.load())<<","<<to_string(sys->log_lastRuntime.load())<<","<<
             to_string(sys->log_worstRuntime.load())<<","<<to_string(sys->log_dlnMissed.load())<<","<<to_string(sys->print_lastRuntime.load())<<","<<to_string(sys->print_worstRuntime.load())<<","<<to_string(sys->print_dlnMissed.load())<<","<<
             to_string(cc->cc_lastRuntime.load())<<","<<to_string(cc->cc_worstRuntime.load())<<","<<to_string(cc->cc_dlnMissed.load())<<","<<to_string(cn->cn_lastRuntime.load())<<","<<to_string(cn->cn_worstRuntime.load())<<","<<
             to_string(cn->cn_dlnMissed.load())<<","<<to_string(sys->bgp_lastRuntime.load())<<","<<to_string(sys->bgp_worstRuntime.load())<<","<<to_string(sys->bgt_lastRuntime.load())<<","<<to_string(sys->bgt_worstRuntime.load())<<","<<endl;

    csvFile.flush();
}
