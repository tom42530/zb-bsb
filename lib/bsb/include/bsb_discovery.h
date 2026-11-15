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
 * départ absente) et les réponses « --- » (octet d'état à 1 : fonction absente du
 * régulateur), et sauvegarde le profil dans la flash NVS.
 */
class BsbDiscovery {
public:
    /**
     * @brief Charge la topologie depuis la mémoire non-volatile (NVS).
     * @param allow_older_version Accepte aussi une topologie mémorisée par un firmware antérieur (même
     *        format binaire, critères de découverte périmés) : repli quand la nouvelle découverte échoue.
     * @return true si une configuration valide a été trouvée en NVS.
     */
    static bool load_from_nvs(BsbCapabilities& out_caps, bool allow_older_version = false);

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
     * @brief Relit sur le bus les valeurs initiales (consignes, modes, températures) d'une topologie
     *        chargée depuis la NVS, sans toucher à la liste des organes ni à la NVS.
     *
     * Ces valeurs initialisent les attributs Zigbee, que la pile rapporte d'elle-même dès qu'elle
     * retrouve le réseau. Sans cette relecture, Home Assistant recevrait à chaque redémarrage
     * l'instantané de la découverte (une consigne vieille de plusieurs semaines, par exemple) avant
     * d'être corrigé quelques secondes plus tard. Si le régulateur ne répond pas, l'instantané est conservé.
     */
    static void refresh_initial_values(BsbBus& bus, BsbCapabilities& caps);

    /**
     * @brief Demande une nouvelle découverte au prochain démarrage (bouton BOOT 3 à 8 s, commande `rescan`).
     *
     * La topologie en place n'est PAS effacée : elle reste le repli si la découverte échoue (PAC coupée
     * à ce moment-là). La demande persiste jusqu'à une découverte réussie.
     */
    static void request_rescan();
    static bool rescan_requested();
    static void clear_rescan_request();

    /**
     * @brief Mémoire durable « cette zone sans sonde filaire est alimentée par une sonde externe ».
     *
     * Le régulateur restitue sur 8740 / 8770 la température que la passerelle lui injecte, exactement
     * comme si une sonde filaire existait. Une découverte ne peut donc pas distinguer les deux : ce
     * drapeau l'empêche de déclarer une sonde filaire fantôme (qui changerait la structure Zigbee).
     */
    static bool external_sensor_used(uint8_t zone_index);
    static void set_external_sensor_used(uint8_t zone_index, bool used);

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
