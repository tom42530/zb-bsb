#pragma once

#include <cstdint>

namespace bsb {

// ============================================================================
// Codes statut Siemens Albatros2 (RVS21 / RVS43 / RVS61 ...)
//
//   Prog 8000 / 8001 : État circuit chauffage 1 / 2
//   Prog 8003        : État eau chaude sanitaire (ECS)
//   Prog 8006        : État pompe à chaleur
//
// Numérotation issue des tables de référence ENUM8000 / ENUM8003 / ENUM8006 du
// projet BSB-LAN (identiques à la documentation Siemens). Ces codes font autorité
// pour savoir ce que fait réellement le régulateur : un circuit en « Limitation
// de température ambiante » (122) ne chauffe pas même si le compresseur tourne
// pour une autre zone, alors qu'un circuit en « Chauffage confort » (114) est
// bien en demande.
// ============================================================================

/** Ce que fait un circuit de chauffage d'après son code statut (8000 / 8001). */
enum class ZoneActivity : uint8_t {
    Unknown,    // Code 0 (« --- », pas encore lu) ou code non répertorié
    Idle,       // Aucune demande de chaleur (arrêt, été, éco, abaissement, limitation ambiance...)
    Restricted, // Demande présente mais suspendue par le régulateur (priorité ECS, protection générateur...)
    Heating     // Le circuit demande de la chaleur (confort, réduit, optimisation, hors-gel actif...)
};

/** Ce que fait la préparation ECS d'après son code statut (8003). */
enum class DhwActivity : uint8_t {
    Unknown,          // Code 0 ou non répertorié
    Idle,             // Ballon chargé, arrêt, verrouillage...
    Charging,         // Charge par le générateur (pompe à chaleur / appoint départ)
    ChargingElectric  // Charge par la résistance électrique du ballon (indépendante du compresseur)
};

/** Ce que fait la pompe à chaleur d'après son code statut (8006). */
enum class HeatPumpActivity : uint8_t {
    Unknown,           // Code 0 ou non répertorié
    Stopped,           // Compresseur à l'arrêt (pas de demande, temps d'arrêt mini, verrouillage, défaut...)
    Compressor,        // Compresseur en marche, production de chaleur
    CompressorAndElectric, // Compresseur + appoint électrique en marche
    ElectricOnly,      // Appoint électrique seul (y compris mode secours)
    Defrost,           // Dégivrage / égouttage : le compresseur ne chauffe pas l'installation
    Cooling            // Rafraîchissement actif ou passif
};

ZoneActivity     classify_zone_status(uint16_t code);
DhwActivity      classify_dhw_status(uint16_t code);
HeatPumpActivity classify_hp_status(uint16_t code);

/** Libellés français (ASCII) des codes statut, pour les journaux de diagnostic. */
const char* get_zone_status_text_fr(uint16_t code);
const char* get_dhw_status_text_fr(uint16_t code);
const char* get_hp_status_text_fr(uint16_t code);

// ============================================================================
// Calcul de l'état de chauffe réel par thermostat (Zone 1, Zone 2, Ballon ECS)
// ============================================================================

/** Instantané de l'état de la pompe à chaleur lu sur le bus BSB. */
struct HeatingInputs {
    // --- Topologie découverte ---
    bool has_zone1            = false;
    bool has_zone2            = false;
    bool has_dhw              = false;
    bool has_compressor_state = false; // Prog 8400 disponible
    bool has_compressor_mod   = false; // Prog 8413 disponible
    bool has_backup_state     = false; // Prog 8402 disponible
    bool has_backup_mod       = false; // Prog 8414 disponible
    bool zone1_has_pump_state = false; // Prog 8730 (ou 8406) exploitable pour la zone 1
    bool zone2_has_pump_state = false; // Prog 8760 exploitable pour la zone 2
    // Le régulateur fournit un code statut pour ce circuit. Tant qu'il n'a pas été relu (code 0), la zone est
    // tenue pour inactive ; s'il n'est pas répertorié, c'est la pompe du circuit qui dit si la zone demande.
    bool zone1_has_status_code = false; // Prog 8000
    bool zone2_has_status_code = false; // Prog 8001

    // --- Générateur ---
    bool     compressor_on  = false;
    uint8_t  compressor_mod = 0; // 0..100 %
    bool     backup_on      = false;
    uint8_t  backup_mod     = 0; // 0..100 %
    uint16_t hp_status      = 0; // Prog 8006

    // --- Températures hydrauliques du générateur (garde de plausibilité physique) ---
    bool  has_hp_temps = false; // Départ (8412) et retour (8410) tous deux disponibles et récents
    float hp_flow_c    = 0.0f;
    float hp_return_c  = 0.0f;

    // --- Consommateurs ---
    uint16_t zone1_status   = 0; // Prog 8000
    uint16_t zone2_status   = 0; // Prog 8001
    uint16_t dhw_status     = 0; // Prog 8003
    bool     zone1_pump_on  = false;
    bool     zone2_pump_on  = false;
    bool     zone1_protection_mode = false; // Prog 700 = Hors-gel
    bool     zone2_protection_mode = false; // Prog 1000 = Hors-gel
};

/** État de chauffe publié vers Zigbee (ThermostatRunningState + PIHeatingDemand). */
struct ThermostatAction {
    bool    heating = false;
    uint8_t demand  = 0; // 0 si inactif, 1..100 % si en chauffe
};

/** Ce sur quoi repose la demande de chaleur (ou son absence) attribuée à une zone. */
enum class ZoneEvidence : uint8_t {
    None,       // Rien d'exploitable : la zone est tenue pour inactive
    StatusCode, // Code statut répertorié (cas normal)
    Pump,       // Code statut absent ou non répertorié : pompe du circuit
    Mode        // Régulateur sans code statut ni pompe observable : mode de la zone
};

struct HeatingOutputs {
    bool             heat_source_on = false; // Le régulateur annonce le compresseur (hors dégivrage) ou l'appoint en marche
    bool             heating_circuits_fed = true; // Faux si le retour est nettement plus chaud que le départ
    // Une zone demande de la chaleur et le régulateur annonce le générateur en marche pour le chauffage,
    // mais les sondes le démentent : seul cas où la garde hydraulique change l'affichage d'un thermostat.
    bool             zone_heating_refuted = false;
    ThermostatAction zone1;
    ThermostatAction zone2;
    ThermostatAction dhw;
    // Origine de la demande retenue pour chaque zone, pour le diagnostic (indépendante de l'état du générateur)
    ZoneEvidence     zone1_evidence = ZoneEvidence::None;
    ZoneEvidence     zone2_evidence = ZoneEvidence::None;
};

/**
 * Écart retour - départ (K) à partir duquel on considère qu'aucune chaleur n'atteint les circuits de
 * chauffage. Lorsqu'un générateur chauffe réellement un circuit, son départ est plus chaud que son
 * retour ; l'inverse franc (au-delà de la tolérance des sondes) est physiquement incompatible.
 */
static constexpr float HEATING_REVERSE_SPREAD_K = 1.0f;

/**
 * @brief Détermine, pour chaque thermostat, s'il reçoit réellement de la chaleur.
 *
 * Un thermostat est « en chauffe » uniquement si :
 *   1. son propre code statut indique une demande active (pas seulement que la PAC tourne). Si ce code
 *      n'est pas répertorié, la pompe du circuit en tient lieu ; sans pompe observable, rien n'est affirmé.
 *      Relevé sur site : code 255, absent des tables de référence, pendant toute la durée d'une consigne
 *      temporaire réglée sur l'afficheur de la PAC, pompe du circuit et compresseur en marche. ET
 *   2. le régulateur annonce que le générateur produit de la chaleur (compresseur hors dégivrage /
 *      rafraîchissement, ou appoint électrique), ET
 *   3. pour une zone de chauffage : la PAC n'est pas en train de charger le ballon ECS
 *      (vanne directionnelle : la chaleur part alors vers le ballon), ET
 *   4. pour une zone de chauffage : les sondes hydrauliques ne contredisent pas le régulateur. Sur les
 *      Alféa (unité extérieure pilotée par le RVS21), l'état compresseur (8400), sa modulation (8413) et
 *      le statut PAC (8006) traduisent la DEMANDE du régulateur : relevé sur une installation réelle,
 *      « compresseur à 100 % » pendant des heures alors que la PAC était à l'arrêt, circuit fermé.
 *      Un retour nettement plus chaud que le départ prouve qu'aucune chaleur n'est délivrée aux zones.
 *      (Non appliqué à l'ECS : pendant une charge du ballon, départ et retour sont quasi égaux.)
 *
 *      Limite connue : si l'unité extérieure s'arrête d'elle-même alors que le départ reste plus chaud que
 *      le retour (eau immobile dans le circuit), la zone reste affichée en chauffe jusqu'à ce que le
 *      régulateur retire sa demande. Relevé sur site le 2026-11-15 : compresseur annoncé en marche de 0 h 58
 *      à 1 h 18, compteur électrique à 12 W ; sur 19 jours, 70 min ainsi affichées à tort pour 40 h de vraie
 *      chauffe. Ce régulateur ne fournit rien qui trahisse cet arrêt : débit (8460) absent, puissance
 *      absorbée (8397) calculée d'après la modulation demandée. Et « la demande monte pendant que le départ
 *      baisse » s'observe aussi compresseur en marche : sur ces 19 jours, en faire une règle retirait
 *      davantage de vraies chauffes que de fausses. Seule une mesure électrique extérieure au bus tranche.
 *
 * Fonction pure (aucun accès matériel) : testable sur PC.
 */
HeatingOutputs compute_heating_state(const HeatingInputs& in);

/**
 * Durée pendant laquelle une mise en chauffe doit se maintenir avant d'être publiée.
 *
 * Les entrées de compute_heating_state() sont lues une à une sur le bus, à plusieurs secondes d'intervalle
 * (codes statut toutes les 15 s, sondes départ / retour toutes les 20 s). Juste après un changement d'état
 * du régulateur, le calcul mêle donc des valeurs d'avant et d'après. Relevé sur site : une zone dite en
 * chauffe pendant 3 s à la reprise d'une charge ECS (statut PAC déjà relu, statuts zone et ECS pas encore),
 * et pendant 20 s après un dégivrage (sondes départ / retour pas encore relues).
 *
 * La durée doit dépasser la plus longue période de scrutation de ces entrées : chacune a alors été relue au
 * moins une fois pendant que le verdict tenait.
 */
static constexpr uint32_t HEATING_CONFIRM_MS = 30000;

/**
 * @brief Filtre de publication de l'état de chauffe d'un thermostat.
 *
 * L'arrêt est publié immédiatement ; une mise en chauffe ne l'est qu'après s'être maintenue
 * HEATING_CONFIRM_MS sans interruption. Le tout premier verdict fait exception : il est publié tel quel,
 * faute de quoi chaque redémarrage du pont en pleine chauffe ferait voir un arrêt de 30 s.
 *
 * Classe sans accès matériel (l'horloge est passée en argument) : testable sur PC.
 */
class HeatingConfirmation {
public:
    /**
     * @param computed Verdict instantané de compute_heating_state() pour ce thermostat
     * @param now_ms   Horloge monotone, en millisecondes
     * @return L'état à publier
     */
    ThermostatAction update(const ThermostatAction& computed, uint64_t now_ms);

    /** Vrai tant qu'une chauffe calculée attend sa confirmation (rien n'est encore publié). */
    bool pending() const { return state_ == State::Pending; }

private:
    enum class State : uint8_t {
        Unprimed, // Aucun verdict reçu depuis le démarrage
        Idle,
        Pending,  // Chauffe calculée, pas encore confirmée
        Heating
    };

    State    state_         = State::Unprimed;
    uint64_t pending_since_ = 0;
};

/**
 * Durée au-delà de laquelle une zone en demande que la garde hydraulique tient pour non alimentée n'est plus
 * un état passager.
 *
 * Deux situations normales font agir la garde pendant quelques dizaines de secondes :
 *  - la mise en route après un arrêt prolongé : la sonde de départ du générateur est alors plus froide que
 *    celle de retour et met de l'ordre d'une minute à repasser au-dessus (relevé sur site le 2026-11-14 à
 *    22:48 : compresseur annoncé en marche, départ 15,6 C et retour 18,5 C, écart redevenu positif 40 à 70 s
 *    plus tard) ;
 *  - la reprise du chauffage à la fin d'une charge ECS : la sonde de retour lit alors environ 1 K de plus que
 *    celle de départ, jusqu'à ce que l'eau des zones revienne (relevé sur site le 2026-11-15 à 11:10 :
 *    compresseur en marche depuis 43 min, départ 54,1 C et retour 55,2 C, garde levée 16 s plus tard).
 * La zone reste affichée inactive pendant ce temps, ce qui est exact : il n'y a rien à signaler.
 * Si la situation dure, c'est l'anomalie que la garde existe pour démasquer : générateur demandé mais à
 * l'arrêt, ou circuit fermé (relevé sur site : pendant des heures).
 */
static constexpr uint32_t UNFED_ZONE_ALERT_MS = 600000;

/**
 * @brief Dit quand la garde hydraulique retire la chauffe à une zone depuis trop longtemps pour un état passager.
 *
 * Classe sans accès matériel (l'horloge est passée en argument) : testable sur PC.
 */
class UnfedZoneWatch {
public:
    /**
     * @param zone_refuted HeatingOutputs::zone_heating_refuted du dernier calcul
     * @param now_ms       Horloge monotone, en millisecondes
     * @return true une seule fois par épisode, quand il dure depuis UNFED_ZONE_ALERT_MS
     */
    bool update(bool zone_refuted, uint64_t now_ms);

    /** La garde agit depuis moins de UNFED_ZONE_ALERT_MS : état passager jusqu'à preuve du contraire. */
    bool in_grace_period() const { return active_ && !alerted_; }

    /** La garde agit depuis plus longtemps : l'anomalie est établie. */
    bool alerted() const { return alerted_; }

private:
    bool     active_  = false;
    bool     alerted_ = false;
    uint64_t since_   = 0;
};

} // namespace bsb
