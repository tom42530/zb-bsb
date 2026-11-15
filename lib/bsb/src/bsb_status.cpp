#include "bsb_status.h"

#include <cstddef>

namespace bsb {

namespace {

struct StatusEntry {
    uint16_t    code;
    uint8_t     activity; // ZoneActivity / DhwActivity / HeatPumpActivity selon la table
    const char* text_fr;
};

template <size_t N>
const StatusEntry* find_status(const StatusEntry (&table)[N], uint16_t code) {
    for (size_t i = 0; i < N; ++i) {
        if (table[i].code == code) {
            return &table[i];
        }
    }
    return nullptr;
}

constexpr uint8_t Z_UNKNOWN    = static_cast<uint8_t>(ZoneActivity::Unknown);
constexpr uint8_t Z_IDLE       = static_cast<uint8_t>(ZoneActivity::Idle);
constexpr uint8_t Z_RESTRICTED = static_cast<uint8_t>(ZoneActivity::Restricted);
constexpr uint8_t Z_HEATING    = static_cast<uint8_t>(ZoneActivity::Heating);

// ----------------------------------------------------------------------------
// Prog 8000 / 8001 — État circuit chauffage (table ENUM8000)
//
// Les codes 22 à 25 (hors-gel installation / hors-gel / arrêt) sont numérotés
// différemment selon les sources (BSB-LAN : 22/23/24, documentation Siemens :
// 23/24/25). Ils sont donc tous classés « inactif » : le hors-gel réellement en
// chauffe est signalé sans ambiguïté par les codes 101 (ambiance) et 117 (départ).
// ----------------------------------------------------------------------------
const StatusEntry ZONE_STATUS[] = {
    {0,   Z_UNKNOWN,    "---"},
    {3,   Z_IDLE,       "Thermostat limiteur declenche"},
    {4,   Z_HEATING,    "Regime manuel actif"},
    {17,  Z_IDLE,       "Post-circulation active"},
    {22,  Z_IDLE,       "Hors-gel installation actif"},
    {23,  Z_IDLE,       "Protection hors-gel active"},
    {24,  Z_IDLE,       "Arret"},
    {25,  Z_IDLE,       "Arret"},
    {56,  Z_HEATING,    "Protection surchauffe active"},
    {101, Z_HEATING,    "Hors-gel ambiance actif"},
    {102, Z_HEATING,    "Sechage controle de dalle actif"},
    {103, Z_RESTRICTED, "Restreint, protection generateur"},
    {104, Z_RESTRICTED, "Restreint, priorite ECS"},
    {105, Z_RESTRICTED, "Restreint, ballon tampon"},
    {106, Z_RESTRICTED, "Regime de chauffe restreint"},
    {107, Z_HEATING,    "Evacuation forcee, ballon tampon"},
    {108, Z_HEATING,    "Evacuation forcee, ECS"},
    {109, Z_HEATING,    "Evacuation forcee, generateur"},
    {110, Z_HEATING,    "Evacuation forcee"},
    {111, Z_HEATING,    "Optimisation enclenchement + chauffe acceleree"},
    {112, Z_HEATING,    "Optimisation a l'enclenchement"},
    {113, Z_HEATING,    "Mise en temperature acceleree"},
    {114, Z_HEATING,    "Chauffage confort"},
    {115, Z_IDLE,       "Optimisation a la coupure"},
    {116, Z_HEATING,    "Chauffage reduit"},
    {117, Z_HEATING,    "Hors-gel depart actif"},
    {118, Z_IDLE,       "Regime d'ete"},
    {119, Z_IDLE,       "Eco journalier actif"},
    {120, Z_IDLE,       "Abaissement reduit"},
    {121, Z_IDLE,       "Abaissement hors-gel"},
    {122, Z_IDLE,       "Limitation temperature ambiante"},
    {248, Z_IDLE,       "Marche continue de la pompe"},
};

constexpr uint8_t D_UNKNOWN  = static_cast<uint8_t>(DhwActivity::Unknown);
constexpr uint8_t D_IDLE     = static_cast<uint8_t>(DhwActivity::Idle);
constexpr uint8_t D_CHARGING = static_cast<uint8_t>(DhwActivity::Charging);
constexpr uint8_t D_ELECTRIC = static_cast<uint8_t>(DhwActivity::ChargingElectric);

// ----------------------------------------------------------------------------
// Prog 8003 — État eau chaude sanitaire (table ENUM8003)
// ----------------------------------------------------------------------------
const StatusEntry DHW_STATUS[] = {
    {0,   D_UNKNOWN,  "---"},
    {3,   D_IDLE,     "Thermostat limiteur declenche"},
    {4,   D_IDLE,     "Regime manuel actif"},
    {17,  D_IDLE,     "Post-circulation active"},
    {24,  D_CHARGING, "Protection hors-gel active"},
    {25,  D_IDLE,     "Arret"},
    {53,  D_IDLE,     "Refroidissement actif"},
    {66,  D_ELECTRIC, "Charge par resistance electrique"},
    {67,  D_CHARGING, "Charge forcee active"},
    {69,  D_CHARGING, "Charge active"},
    {70,  D_IDLE,     "Charge terminee, temp. max ballon"},
    {71,  D_IDLE,     "Charge terminee, temp. max de charge"},
    {75,  D_IDLE,     "Charge terminee"},
    {77,  D_IDLE,     "Refroidissement par collecteur"},
    {78,  D_IDLE,     "Refroidissement par generateur/circuits"},
    {79,  D_IDLE,     "Protection de decharge active"},
    {80,  D_IDLE,     "Limitation duree de charge active"},
    {81,  D_IDLE,     "Charge verrouillee"},
    {82,  D_IDLE,     "Verrouillage de charge actif"},
    {83,  D_CHARGING, "Charge forcee, temp. max ballon"},
    {84,  D_CHARGING, "Charge forcee, temp. max de charge"},
    {85,  D_CHARGING, "Charge forcee, consigne legionelles"},
    {86,  D_CHARGING, "Charge forcee, consigne nominale"},
    {87,  D_ELECTRIC, "Charge electrique, consigne legionelles"},
    {88,  D_ELECTRIC, "Charge electrique, consigne nominale"},
    {89,  D_ELECTRIC, "Charge electrique, consigne reduite"},
    {90,  D_ELECTRIC, "Charge electrique, consigne hors-gel"},
    {91,  D_ELECTRIC, "Resistance electrique liberee"},
    {92,  D_CHARGING, "Push, consigne legionelles"},
    {93,  D_CHARGING, "Push, consigne nominale"},
    {94,  D_CHARGING, "Push actif"},
    {95,  D_CHARGING, "En charge, consigne legionelles"},
    {96,  D_CHARGING, "En charge, consigne nominale"},
    {97,  D_CHARGING, "En charge, consigne reduite"},
    {98,  D_IDLE,     "Charge terminee, temp. legionelles"},
    {99,  D_IDLE,     "Charge terminee, temp. nominale"},
    {100, D_IDLE,     "Charge terminee, temp. reduite"},
    {199, D_CHARGING, "Soutirage"},
    {200, D_IDLE,     "Pret"},
    {201, D_CHARGING, "Charge de maintien"},
};

constexpr uint8_t H_UNKNOWN  = static_cast<uint8_t>(HeatPumpActivity::Unknown);
constexpr uint8_t H_STOPPED  = static_cast<uint8_t>(HeatPumpActivity::Stopped);
constexpr uint8_t H_COMP     = static_cast<uint8_t>(HeatPumpActivity::Compressor);
constexpr uint8_t H_COMP_EL  = static_cast<uint8_t>(HeatPumpActivity::CompressorAndElectric);
constexpr uint8_t H_ELECTRIC = static_cast<uint8_t>(HeatPumpActivity::ElectricOnly);
constexpr uint8_t H_DEFROST  = static_cast<uint8_t>(HeatPumpActivity::Defrost);
constexpr uint8_t H_COOLING  = static_cast<uint8_t>(HeatPumpActivity::Cooling);

// ----------------------------------------------------------------------------
// Prog 8006 — État pompe à chaleur (table ENUM8006)
//
// Les états de limitation (32, 33, 37, 40..43...) ne disent pas si le compresseur
// tourne : ils restent « Unknown » et c'est le Prog 8400 (état compresseur) qui tranche.
// ----------------------------------------------------------------------------
const StatusEntry HP_STATUS[] = {
    {0,   H_UNKNOWN,  "---"},
    {2,   H_STOPPED,  "Defaut"},
    {10,  H_STOPPED,  "Verrouille"},
    {17,  H_STOPPED,  "Post-circulation active"},
    {23,  H_STOPPED,  "Hors-gel installation actif"},
    {24,  H_UNKNOWN,  "Protection hors-gel active"},
    {25,  H_STOPPED,  "Arret"},
    {26,  H_ELECTRIC, "Mode secours"},
    {27,  H_STOPPED,  "Verrouille, externe"},
    {29,  H_STOPPED,  "Haute pression en fonctionnement PAC"},
    {30,  H_STOPPED,  "Controleur de debit source"},
    {31,  H_STOPPED,  "Pressostat source"},
    {32,  H_UNKNOWN,  "Limitation gaz chauds compresseur 1"},
    {33,  H_UNKNOWN,  "Limitation gaz chauds compresseur 2"},
    {34,  H_STOPPED,  "Limitation temp. de coupure max"},
    {35,  H_STOPPED,  "Temps d'arret min. compresseur actif"},
    {36,  H_STOPPED,  "Compensation excedent de chaleur"},
    {37,  H_UNKNOWN,  "Temps de limitation actif"},
    {38,  H_COMP,     "Temps de marche min. compresseur actif"},
    {39,  H_COMP,     "Compensation deficit de chaleur"},
    {40,  H_UNKNOWN,  "Limitation ecart condenseur max"},
    {41,  H_UNKNOWN,  "Limitation ecart condenseur min"},
    {42,  H_UNKNOWN,  "Limitation ecart evaporateur max"},
    {43,  H_UNKNOWN,  "Limitation ecart evaporateur min"},
    {44,  H_COMP_EL,  "Compresseur et appoint electrique en marche"},
    {45,  H_COMP,     "Compresseurs 1 et 2 en marche"},
    {46,  H_COMP,     "Compresseur 1 en marche"},
    {47,  H_COMP,     "Compresseur 2 en marche"},
    {48,  H_UNKNOWN,  "Hors-gel pompe a chaleur"},
    {49,  H_STOPPED,  "Pre-circulation active"},
    {50,  H_STOPPED,  "Liberee, compresseur pret"},
    {51,  H_STOPPED,  "Pas de demande"},
    {125, H_DEFROST,  "Degivrage actif"},
    {126, H_DEFROST,  "Egouttage"},
    {127, H_COOLING,  "Rafraichissement actif"},
    {128, H_COOLING,  "Rafraichissement passif"},
    {129, H_DEFROST,  "Refroidissement evaporateur"},
    {130, H_UNKNOWN,  "Prechauffage pour degivrage"},
    {137, H_COMP,     "Regime de chauffe"},
    {139, H_STOPPED,  "Limitation temp. de coupure min"},
    {145, H_COOLING,  "Limitation temp. ext. max rafraichissement"},
    {176, H_STOPPED,  "Verrouille, temperature exterieure"},
    {180, H_STOPPED,  "Triphase asymetrique"},
    {181, H_STOPPED,  "Basse pression"},
    {182, H_STOPPED,  "Surcharge ventilateur"},
    {183, H_STOPPED,  "Surcharge compresseur 1"},
    {184, H_STOPPED,  "Surcharge compresseur 2"},
    {185, H_STOPPED,  "Surcharge pompe source"},
    {186, H_STOPPED,  "Controleur de debit consommateurs"},
    {187, H_STOPPED,  "Limite d'utilisation temp. ext. min"},
    {188, H_STOPPED,  "Limite d'utilisation temp. ext. max"},
    {189, H_STOPPED,  "Limitation temp. source min (eau)"},
    {190, H_STOPPED,  "Limitation temp. source min (eau glycolee)"},
    {191, H_STOPPED,  "Limitation temp. source max"},
    {192, H_DEFROST,  "Degivrage force par compresseur"},
    {193, H_DEFROST,  "Degivrage force par ventilateur"},
    {194, H_DEFROST,  "Degivrage par compresseur"},
    {195, H_DEFROST,  "Degivrage par ventilateur"},
    {196, H_COOLING,  "Limitation temp. source min rafraichissement"},
    {197, H_ELECTRIC, "Appoint electrique en marche"},
    {198, H_STOPPED,  "Verrouille, mode eco"},
    {207, H_COOLING,  "Temps de marche min. compresseur, rafraichissement"},
    {208, H_COOLING,  "Compresseurs 1 et 2 en marche, rafraichissement"},
    {209, H_COOLING,  "Compresseur 1 en marche, rafraichissement"},
    {210, H_COOLING,  "Compresseur 2 en marche, rafraichissement"},
    {235, H_STOPPED,  "Pression d'eau trop basse"},
    {242, H_UNKNOWN,  "Destratification active"},
    {246, H_STOPPED,  "Sous-tension reseau"},
    {254, H_STOPPED,  "Recuperation fluide frigorigene, manuel"},
    {256, H_STOPPED,  "Recuperation fluide frigorigene"},
    {257, H_UNKNOWN,  "Temporisation demarrage degivrage"},
    {258, H_STOPPED,  "Compresseur verrouille"},
    {259, H_STOPPED,  "Verrouille, temp. source max"},
    {260, H_STOPPED,  "Verrouille, temp. source min"},
    {261, H_STOPPED,  "Verrouille, temp. retour max"},
    {262, H_STOPPED,  "Verrouille, temp. retour min"},
    {263, H_STOPPED,  "Verrouille, temp. depart max"},
    {264, H_STOPPED,  "Verrouille, temp. depart min"},
    {265, H_STOPPED,  "Verrouille, temp. condenseur max"},
    {266, H_STOPPED,  "Verrouille, temp. evaporateur min"},
    {267, H_STOPPED,  "Verrouille, temp. gaz chauds max"},
    {268, H_UNKNOWN,  "Limitation temp. evaporation min"},
    {269, H_UNKNOWN,  "Limitation temp. condensation max"},
    {270, H_UNKNOWN,  "Limitation temp. evaporation max"},
};

uint8_t clamp_percent(uint8_t value) {
    if (value < 1) return 1;
    if (value > 100) return 100;
    return value;
}

} // namespace

ZoneActivity classify_zone_status(uint16_t code) {
    const StatusEntry* e = find_status(ZONE_STATUS, code);
    return e ? static_cast<ZoneActivity>(e->activity) : ZoneActivity::Unknown;
}

DhwActivity classify_dhw_status(uint16_t code) {
    const StatusEntry* e = find_status(DHW_STATUS, code);
    return e ? static_cast<DhwActivity>(e->activity) : DhwActivity::Unknown;
}

HeatPumpActivity classify_hp_status(uint16_t code) {
    const StatusEntry* e = find_status(HP_STATUS, code);
    return e ? static_cast<HeatPumpActivity>(e->activity) : HeatPumpActivity::Unknown;
}

const char* get_zone_status_text_fr(uint16_t code) {
    const StatusEntry* e = find_status(ZONE_STATUS, code);
    return e ? e->text_fr : "Autre etat circuit chauffage";
}

const char* get_dhw_status_text_fr(uint16_t code) {
    const StatusEntry* e = find_status(DHW_STATUS, code);
    return e ? e->text_fr : "Autre etat ECS";
}

const char* get_hp_status_text_fr(uint16_t code) {
    const StatusEntry* e = find_status(HP_STATUS, code);
    return e ? e->text_fr : "Autre etat PAC";
}

HeatingOutputs compute_heating_state(const HeatingInputs& in) {
    HeatingOutputs out;
    const HeatPumpActivity hp = classify_hp_status(in.hp_status);

    // 1. Le générateur produit-il de la chaleur ? Les états matériels lus sur le bus (8400 / 8402)
    //    font foi ; le code statut PAC ne sert de repli que s'ils ne sont pas disponibles.
    //    Une modulation nulle ne prouve pas l'arrêt : certains régulateurs y répondent « --- » (lu 0 %)
    //    en permanence. Sans état matériel, modulation et code statut se complètent donc.
    bool compressor_on = false;
    if (in.has_compressor_state) {
        compressor_on = in.compressor_on;
    } else {
        compressor_on = (in.has_compressor_mod && in.compressor_mod > 0) ||
                        hp == HeatPumpActivity::Compressor || hp == HeatPumpActivity::CompressorAndElectric;
    }

    bool electric_on = false;
    if (in.has_backup_state) {
        electric_on = in.backup_on;
    } else {
        electric_on = (in.has_backup_mod && in.backup_mod > 0) ||
                      hp == HeatPumpActivity::CompressorAndElectric || hp == HeatPumpActivity::ElectricOnly;
    }

    // En dégivrage ou en rafraîchissement, le compresseur tourne mais ne chauffe pas l'installation
    const bool compressor_heats = compressor_on &&
                                  hp != HeatPumpActivity::Defrost &&
                                  hp != HeatPumpActivity::Cooling;
    out.heat_source_on = compressor_heats || electric_on;

    uint8_t source_demand = 0;
    if (out.heat_source_on) {
        if (compressor_heats && in.has_compressor_mod && in.compressor_mod > 0) {
            source_demand = clamp_percent(in.compressor_mod);
        } else if (electric_on && in.has_backup_mod && in.backup_mod > 0) {
            source_demand = clamp_percent(in.backup_mod);
        } else {
            source_demand = 100; // Générateur tout-ou-rien ou modulation non disponible
        }
    }

    // 2. Ballon ECS
    bool dhw_takes_generator = false;
    if (in.has_dhw) {
        const DhwActivity dhw = classify_dhw_status(in.dhw_status);
        if (dhw == DhwActivity::ChargingElectric) {
            out.dhw.heating = true;
            out.dhw.demand  = 100;
        } else if (dhw == DhwActivity::Charging && out.heat_source_on) {
            out.dhw.heating     = true;
            out.dhw.demand      = source_demand;
            dhw_takes_generator = true;
        }
    }

    // 3. Garde de plausibilité physique pour les circuits de chauffage. Les indicateurs du régulateur
    //    (8400 / 8413 / 8006) traduisent sa demande à l'unité extérieure, pas son fonctionnement réel :
    //    si le retour est nettement plus chaud que le départ, aucune chaleur n'atteint les zones
    //    (générateur à l'arrêt malgré la demande, absence de circulation, ou dégivrage).
    out.heating_circuits_fed =
        !(in.has_hp_temps && (in.hp_return_c - in.hp_flow_c) >= HEATING_REVERSE_SPREAD_K);

    // 4. Zones de chauffage
    const auto zone_action = [&](bool present, uint16_t status, bool has_status_code, bool has_pump_state,
                                 bool pump_on, bool protection_mode, ZoneEvidence& evidence) {
        ThermostatAction action;
        evidence = ZoneEvidence::None;
        if (!present) {
            return action;
        }

        // 4a. La zone demande-t-elle de la chaleur ? (indépendamment de l'état du générateur)
        bool demand = false;
        const ZoneActivity activity = classify_zone_status(status);
        if (activity != ZoneActivity::Unknown) {
            evidence = ZoneEvidence::StatusCode;
            demand   = (activity == ZoneActivity::Heating);
        } else if (has_status_code && status == 0) {
            // Code pas encore relu depuis le démarrage : rien n'est affirmé, même si la pompe tourne
        } else if (has_pump_state) {
            // Code non répertorié, ou régulateur sans code statut : la pompe du circuit dit si le régulateur
            // alimente la zone (il l'arrête dès qu'elle n'a plus besoin de chaleur)
            evidence = ZoneEvidence::Pump;
            demand   = pump_on;
        } else if (!has_status_code) {
            // Régulateur sans code statut ni pompe observable : repli sur le mode de la zone
            evidence = ZoneEvidence::Mode;
            demand   = !protection_mode;
        }
        // Dernier cas, code non répertorié sans pompe observable : rien ne permet de trancher (ZoneEvidence::None)

        // 4b. Cette demande est-elle servie ?
        if (!demand || !out.heat_source_on || dhw_takes_generator) {
            return action;
        }
        if (!out.heating_circuits_fed) {
            out.zone_heating_refuted = true; // La demande existe, mais aucune chaleur n'arrive à la zone
            return action;
        }
        action.heating = true;
        action.demand  = source_demand;
        return action;
    };

    out.zone1 = zone_action(in.has_zone1, in.zone1_status, in.zone1_has_status_code, in.zone1_has_pump_state,
                            in.zone1_pump_on, in.zone1_protection_mode, out.zone1_evidence);
    out.zone2 = zone_action(in.has_zone2, in.zone2_status, in.zone2_has_status_code, in.zone2_has_pump_state,
                            in.zone2_pump_on, in.zone2_protection_mode, out.zone2_evidence);
    return out;
}

ThermostatAction HeatingConfirmation::update(const ThermostatAction& computed, uint64_t now_ms) {
    if (!computed.heating) {
        state_ = State::Idle;
        return ThermostatAction{};
    }

    switch (state_) {
        case State::Unprimed:
            state_ = State::Heating; // Premier verdict : aucun état antérieur à protéger
            break;
        case State::Idle:
            state_         = State::Pending;
            pending_since_ = now_ms;
            break;
        case State::Pending:
            if (now_ms >= pending_since_ && (now_ms - pending_since_) >= HEATING_CONFIRM_MS) {
                state_ = State::Heating;
            }
            break;
        case State::Heating:
            break;
    }
    return (state_ == State::Heating) ? computed : ThermostatAction{};
}

bool UnfedZoneWatch::update(bool zone_refuted, uint64_t now_ms) {
    if (!zone_refuted) {
        active_  = false;
        alerted_ = false;
        return false;
    }
    if (!active_) {
        active_ = true;
        since_  = now_ms;
        return false;
    }
    if (!alerted_ && now_ms >= since_ && (now_ms - since_) >= UNFED_ZONE_ALERT_MS) {
        alerted_ = true;
        return true;
    }
    return false;
}

} // namespace bsb
