#pragma once

#if defined(ESP_PLATFORM)

#include "bsb_types.h"
#include "bsb_bus.h"

namespace bsb {

/**
 * @brief Moteur d'Auto-Découverte de la topologie PAC et persistance NVS.
 *
 * Scanne le bus BSB au premier démarrage (ou sur demande via le bouton BOOT),
 * identifie les organes réellement connectés en filtrant les valeurs sentinelles
 * des sondes absentes (ex: 50.0°C pour sonde d'ambiance absente, 0.0°C pour sonde
 * départ absente, ou flag_byte != 0), et sauvegarde le profil dans la flash NVS.
 */
class BsbDiscovery {
public:
    /**
     * @brief Charge la topologie depuis la mémoire non-volatile (NVS).
     * @return true si une configuration valide a été trouvée en NVS.
     */
    static bool load_from_nvs(BsbCapabilities& out_caps);

    /**
     * @brief Sauvegarde la topologie découverte en mémoire non-volatile (NVS).
     */
    static bool save_to_nvs(const BsbCapabilities& caps);

    /**
     * @brief Efface la topologie mémorisée en NVS (force un nouveau scan au prochain boot).
     */
    static bool clear_nvs();

    /**
     * @brief Exécute un scan complet du bus BSB pour découvrir les fonctionnalités de la PAC.
     * @param bus      Instance active du bus BSB
     * @param out_caps Structure remplie avec les capacités détectées et valeurs initiales
     * @return true si le régulateur PAC a répondu et qu'au moins un organe a été découvert
     */
    static bool scan_heat_pump(BsbBus& bus, BsbCapabilities& out_caps);

    /**
     * @brief Affiche un rapport détaillé des fonctionnalités découvertes dans les logs.
     */
    static void log_capabilities(const BsbCapabilities& caps);

    /**
     * @brief Enregistre dans l'ordonnanceur du bus BSB uniquement les paramètres
     *        correspondant aux équipements réellement découverts.
     */
    static void register_discovered_polls(BsbBus& bus, const BsbCapabilities& caps);
};

} // namespace bsb

#endif // defined(ESP_PLATFORM)
