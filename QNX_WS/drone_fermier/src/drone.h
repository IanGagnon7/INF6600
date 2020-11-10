#ifndef __DRONE_H__
#define __DRONE_H__

//Structure pour représenter comment un champs doit être photographier
typedef struct
{
    int nbCoord;
    coord_t* coords;
} champs_t;

//position obtenue du système continue
typedef struct{
    double x;
    double y;
    double z;
    double vh;//vitesse horizontale
    double vv;//vitesse verticale
} position_t;
//actions qui doivent être effectués lorsque ControleurNavigation atteint sa destination
enum Action { act_changeDestination, act_chargeBatterie, act_rechargeMission };

//valeurs globales pour le ControleurCamera
struct ControleurCamera{
    std::atomic<bool> cycleActif;
    std::atomic<coord_t> position_derniere_photo;
    unsigned long long tailleMemoire;
    std::atomic<unsigned long long> memoireOffset;
    std::atomic<bool> photoWait;
    std::atomic<bool> photoDest;
    std::atomic<bool> champsFini;
    std::atomic<bool> takelastpicture;
    std::atomic<unsigned int> photoManquee;
    time_t timestamp;
    std::atomic<double> cc_worstRuntime;
	std::atomic<double> cc_lastRuntime;
	std::atomic<unsigned int> cc_dlnMissed;
};
//valeurs globales pour le ControleurNavigation
struct ControleurNavigation{
    std::atomic<bool> paused;
    std::atomic<int> destinationAtteinteAction;
    std::atomic<coord_t> destination;
    champs_t champs_actif;
    std::atomic<double> cible_vitesse;
    std::atomic<double> cible_altitude;
    position_t gps_lecture;
    std::atomic<double> cn_worstRuntime;
	std::atomic<double> cn_lastRuntime;
	std::atomic<unsigned int> cn_dlnMissed;
};
//valeurs globales pour le ControleurMission
struct ControleurMission{
    //état de la mission précédente à sauvegarder pour changervers la mission prioritaire
    std::atomic<bool> cameraSaved_cycleActif;
    std::atomic<bool> navSaved_paused;
    std::atomic<int> navSaved_destinationAtteinteAction;
    std::atomic<coord_t> navSaved_destination;
    std::atomic<coord_t> saved_position;
};

// Structure de donnée utilisée pour les différentes tâches
struct TaskData {
    struct ControleurCamera* p_ctrlCamera;
    struct ControleurNavigation* p_ctrlNavigation;
    struct ControleurMission* p_ctrlMission;
};

struct TaskData* get_gdata();

#endif
