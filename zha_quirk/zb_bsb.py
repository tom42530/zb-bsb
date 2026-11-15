"""
ZHA Custom Quirk V2 pour Atlantic-Siemens ZB-BSB (Pompe à Chaleur Siemens RVS21).

Ce fichier doit être placé dans le dossier configuré sous Home Assistant :
  /config/zha_quirks/zb_bsb.py

Il implémente :
1. La conversion des codes numériques AnalogInput (0x000C) des statuts PAC, ECS et Zones
   en capteurs d'état textuels enrichis (IntEnum).
2. L'entité Thermostat Zigbee enrichie pour la Zone de Chauffage (Zone 1 - EP1) :
   * Modes HVAC : Éteint (off), Chauffage (heat), Automatique (auto)
   * Préréglages : Confort (comfort), Éco (eco), Auto (auto), Hors-gel (frost_protection)
   * Bornes température : 7.0 °C à 30.0 °C
3. Le Ballon Eau Chaude Sanitaire (ECS - EP6) :
   * Température réelle du ballon (sensor)
   * Consigne nominale ajustable 30-65 °C (number)
   * Marche / Arrêt de l'ECS (switch)
   * Statut ECS enrichi : En charge, Température atteinte, etc. (sensor)
"""

from __future__ import annotations

from enum import IntEnum
import functools
import logging

from zhaquirks.builder import (
    EntityPlatform,
    EntityType,
    QuirkBuilder,
)
from zigpy.zcl.clusters.general import AnalogInput, OnOff
from zigpy.zcl.clusters.hvac import (
    SystemMode,
    Thermostat as ThermostatCluster,
)

_LOGGER = logging.getLogger(__name__)


# ============================================================================
# Enumérateurs de Statuts BSB Siemens Robustes (PAC, Zones Chauffage, Ballon ECS)
# ============================================================================

class RobustIntEnum(IntEnum):
    """IntEnum robuste évitant toute exception ValueError dans Home Assistant.

    Note : Home Assistant valide que `state in self.options`, où `self.options` est construit
    statiquement à partir des membres déclarés de l'Enum. Si un code inconnu est reçu,
    on retourne le membre statique `Autre_code_statut` (255) qui fait partie de `self.options`.
    """

    @classmethod
    def _missing_(cls, value):
        try:
            val_int = int(round(float(value)))
        except (ValueError, TypeError):
            val_int = 255
        for member in cls:
            if member.value == val_int:
                return member
        _LOGGER.warning("ZB-BSB: Code statut BSB non répertorié reçu pour %s : %s", cls.__name__, value)
        return cls(255)


class PacStatusEnum(RobustIntEnum):
    """Prog 8006 - Etat pompe a chaleur (table Siemens ENUM8006)."""

    Inactif = 0
    Defaut = 2
    Verrouille = 10
    Post_circulation_active = 17
    Hors_gel_installation_actif = 23
    Protection_hors_gel_active = 24
    Arret = 25
    Mode_secours = 26
    Verrouille_externe = 27
    Haute_pression = 29
    Controleur_debit_source = 30
    Pressostat_source = 31
    Limitation_gaz_chauds_compresseur_1 = 32
    Limitation_gaz_chauds_compresseur_2 = 33
    Limitation_temp_coupure_max = 34
    Temps_arret_min_compresseur = 35
    Compensation_excedent_chaleur = 36
    Temps_limitation_actif = 37
    Temps_marche_min_compresseur = 38
    Compensation_deficit_chaleur = 39
    Limitation_ecart_condenseur_max = 40
    Limitation_ecart_condenseur_min = 41
    Limitation_ecart_evaporateur_max = 42
    Limitation_ecart_evaporateur_min = 43
    Compresseur_et_appoint_en_marche = 44
    Compresseurs_1_et_2_en_marche = 45
    Compresseur_1_en_marche = 46
    Compresseur_2_en_marche = 47
    Hors_gel_pompe_a_chaleur = 48
    Pre_circulation_active = 49
    Liberee_compresseur_pret = 50
    Pas_de_demande = 51
    Degivrage_actif = 125
    Egouttage = 126
    Rafraichissement_actif = 127
    Rafraichissement_passif = 128
    Refroidissement_evaporateur = 129
    Prechauffage_degivrage = 130
    Regime_de_chauffe = 137
    Limitation_temp_coupure_min = 139
    Limitation_temp_ext_max_rafraichissement = 145
    Verrouille_temperature_exterieure = 176
    Triphase_asymetrique = 180
    Basse_pression = 181
    Surcharge_ventilateur = 182
    Surcharge_compresseur_1 = 183
    Surcharge_compresseur_2 = 184
    Surcharge_pompe_source = 185
    Controleur_debit_consommateurs = 186
    Limite_utilisation_temp_ext_min = 187
    Limite_utilisation_temp_ext_max = 188
    Limitation_temp_source_min_eau = 189
    Limitation_temp_source_min_eau_glycolee = 190
    Limitation_temp_source_max = 191
    Degivrage_force_compresseur = 192
    Degivrage_force_ventilateur = 193
    Degivrage_par_compresseur = 194
    Degivrage_par_ventilateur = 195
    Limitation_temp_source_min_rafraichissement = 196
    Appoint_electrique_en_marche = 197
    Verrouille_mode_eco = 198
    Temps_marche_min_compresseur_rafraichissement = 207
    Compresseurs_1_et_2_rafraichissement = 208
    Compresseur_1_rafraichissement = 209
    Compresseur_2_rafraichissement = 210
    Pression_eau_trop_basse = 235
    Destratification_active = 242
    Sous_tension_reseau = 246
    Recuperation_fluide_manuelle = 254
    Autre_code_statut = 255
    Recuperation_fluide = 256
    Temporisation_demarrage_degivrage = 257
    Compresseur_verrouille = 258
    Verrouille_temp_source_max = 259
    Verrouille_temp_source_min = 260
    Verrouille_temp_retour_max = 261
    Verrouille_temp_retour_min = 262
    Verrouille_temp_depart_max = 263
    Verrouille_temp_depart_min = 264
    Verrouille_temp_condenseur_max = 265
    Verrouille_temp_evaporateur_min = 266
    Verrouille_temp_gaz_chauds_max = 267
    Limitation_temp_evaporation_min = 268
    Limitation_temp_condensation_max = 269
    Limitation_temp_evaporation_max = 270


class ZoneStatusEnum(RobustIntEnum):
    """Prog 8000 / 8001 - Etat circuit chauffage (table Siemens ENUM8000).

    Seuls les etats 4, 56, 101, 102, 107 a 114, 116 et 117 correspondent a une demande de chaleur :
    le firmware n'annonce la zone "en chauffe" que dans ces etats, generateur en marche.
    Les codes 22 a 25 (hors-gel / arret) different selon les generations de regulateurs.
    """

    Inactif = 0
    Thermostat_limiteur_declenche = 3
    Regime_manuel_actif = 4
    Post_circulation_active = 17
    Hors_gel_installation_actif = 22
    Protection_hors_gel_active = 23
    Arret_ou_hors_gel = 24
    Arret = 25
    Protection_surchauffe_active = 56
    Hors_gel_ambiance_actif = 101
    Sechage_dalle_actif = 102
    Restreint_protection_generateur = 103
    Restreint_priorite_ECS = 104
    Restreint_ballon_tampon = 105
    Chauffage_restreint = 106
    Evacuation_forcee_tampon = 107
    Evacuation_forcee_ECS = 108
    Evacuation_forcee_generateur = 109
    Evacuation_forcee = 110
    Optimisation_enclenchement_et_chauffe_acceleree = 111
    Optimisation_enclenchement = 112
    Chauffe_acceleree = 113
    Chauffage_confort = 114
    Optimisation_coupure = 115
    Chauffage_reduit = 116
    Hors_gel_depart_actif = 117
    Regime_ete = 118
    Eco_journalier_actif = 119
    Abaissement_reduit = 120
    Abaissement_hors_gel = 121
    Limitation_temperature_ambiante = 122
    Marche_continue_pompe = 248
    Autre_code_statut = 255


class DhwStatusEnum(RobustIntEnum):
    """Prog 8003 - Etat eau chaude sanitaire (table Siemens ENUM8003)."""

    Inactif = 0
    Thermostat_limiteur_declenche = 3
    Regime_manuel_actif = 4
    Post_circulation_active = 17
    Protection_hors_gel_active = 24
    Arret = 25
    Refroidissement_actif = 53
    Charge_resistance_electrique = 66
    Charge_forcee_active = 67
    Charge_active = 69
    Charge_terminee_temp_max_ballon = 70
    Charge_terminee_temp_max_charge = 71
    Charge_terminee = 75
    Refroidissement_par_collecteur = 77
    Refroidissement_par_generateur = 78
    Protection_decharge_active = 79
    Limitation_duree_charge_active = 80
    Charge_verrouillee = 81
    Verrouillage_charge_actif = 82
    Charge_forcee_temp_max_ballon = 83
    Charge_forcee_temp_max_charge = 84
    Charge_forcee_consigne_legionelles = 85
    Charge_forcee_consigne_nominale = 86
    Charge_electrique_consigne_legionelles = 87
    Charge_electrique_consigne_nominale = 88
    Charge_electrique_consigne_reduite = 89
    Charge_electrique_consigne_hors_gel = 90
    Resistance_electrique_liberee = 91
    Push_consigne_legionelles = 92
    Push_consigne_nominale = 93
    Push_actif = 94
    En_charge_consigne_legionelles = 95
    En_charge_consigne_nominale = 96
    En_charge_consigne_reduite = 97
    Charge_terminee_temp_legionelles = 98
    Charge_terminee_temp_nominale = 99
    Charge_terminee_temp_reduite = 100
    Soutirage = 199
    Pret = 200
    Charge_de_maintien = 201
    Autre_code_statut = 255


# ============================================================================
# 1. Enregistrement des Capteurs de Statut IntEnum (Quirks V2 Builder)
# ============================================================================

(
    QuirkBuilder("Atlantic-Siemens", "ZB-BSB")
    # Statut Zone 2 (Endpoint 2)
    .enum(
        attribute_name="present_value",
        enum_class=ZoneStatusEnum,
        cluster_id=AnalogInput.cluster_id,
        endpoint_id=2,
        entity_platform=EntityPlatform.SENSOR,
        entity_type=EntityType.DIAGNOSTIC,
        translation_key="code_statut_zone_2",
        fallback_name="Code Statut Zone 2",
    )
    # Statut PAC Générale (Endpoint 3)
    .enum(
        attribute_name="present_value",
        enum_class=PacStatusEnum,
        cluster_id=AnalogInput.cluster_id,
        endpoint_id=3,
        entity_platform=EntityPlatform.SENSOR,
        entity_type=EntityType.DIAGNOSTIC,
        translation_key="code_statut_pac",
        fallback_name="Code Statut PAC",
    )
    # Statut Zone 1 (Endpoint 4)
    .enum(
        attribute_name="present_value",
        enum_class=ZoneStatusEnum,
        cluster_id=AnalogInput.cluster_id,
        endpoint_id=4,
        entity_platform=EntityPlatform.SENSOR,
        entity_type=EntityType.DIAGNOSTIC,
        translation_key="code_statut_zone_1",
        fallback_name="Code Statut Zone 1",
    )
    # Statut Ballon ECS (Endpoint 6)
    .enum(
        attribute_name="present_value",
        enum_class=DhwStatusEnum,
        cluster_id=AnalogInput.cluster_id,
        endpoint_id=6,
        entity_platform=EntityPlatform.SENSOR,
        entity_type=EntityType.DIAGNOSTIC,
        translation_key="code_statut_ecs",
        fallback_name="Code Statut ECS",
    )
    # Commutateur d'antenne Zigbee externe U.FL sur Endpoint 7 (OFF par défaut = céramique interne)
    .adds(OnOff.cluster_id, endpoint_id=7)
    .add_to_registry()
)


# ============================================================================
# 2. Entité Thermostat Professionnelle (Zones 1 & 2 + Ballon ECS EP6)
# ============================================================================

try:
    from zha.application.platforms import (
        ClusterMatch,
        PlatformFeatureGroup,
        register_entity,
    )
    from zha.application.platforms.climate import Thermostat
    from zha.application.platforms.climate.const import (
        ClimateEntityFeature,
        HVACMode,
        Preset,
        ZCL_TEMP,
    )
    from zha.application.helpers import write_attributes_safe

    PRESET_COMFORT = Preset.COMFORT if hasattr(Preset, "COMFORT") else "comfort"
    PRESET_ECO = Preset.ECO if hasattr(Preset, "ECO") else "eco"
    PRESET_FROST = "frost_protection"
    PRESET_AUTO = "auto"
    # Valeur « mesure indisponible » d'une température ZCL sur 16 bits signés (0x8000)
    ZCL_INVALID_TEMPERATURE = -32768

    @register_entity(ThermostatCluster.cluster_id)
    class AtlanticSiemensThermostat(Thermostat):
        """Entité Thermostat pour PAC Atlantic/Siemens ZB-BSB (Chauffage & Ballon ECS)."""

        _cluster_match = ClusterMatch(
            server_clusters=frozenset({ThermostatCluster.cluster_id}),
            manufacturers=frozenset({"Atlantic-Siemens"}),
            models=frozenset({"ZB-BSB"}),
            feature_priority=(PlatformFeatureGroup.THERMOSTAT_FAN, 2),
        )

        @property
        def is_dhw(self) -> bool:
            """Retourne True si cet endpoint correspond au Ballon ECS (Endpoint 6)."""
            for attr in ("_endpoint", "endpoint"):
                ep = getattr(self, attr, None)
                if ep is not None:
                    ep_id = getattr(ep, "endpoint_id", None) or getattr(ep, "id", None)
                    if ep_id is not None:
                        return ep_id == 6
            for cluster_attr in ("_cluster", "cluster"):
                cl = getattr(self, cluster_attr, None)
                if cl is not None:
                    ep = getattr(cl, "endpoint", None)
                    if ep is not None:
                        ep_id = getattr(ep, "endpoint_id", None) or getattr(ep, "id", None)
                        if ep_id is not None:
                            return ep_id == 6
            # Fallback par les bornes ou valeurs de consigne ECS (30 - 65 °C vs 7 - 30 °C)
            for attr in ("_abs_min_heat_setpoint_limit", "_min_heat_setpoint_limit"):
                val = getattr(self, attr, None)
                if val is not None and val >= 2500:
                    return True
            return False

        def recompute_capabilities(self) -> None:
            """Configure les capacités et préréglages supportés."""
            super().recompute_capabilities()
            if self.is_dhw:
                self._presets = [
                    PRESET_COMFORT,
                    PRESET_ECO,
                    PRESET_FROST,
                ]
            else:
                self._presets = [
                    PRESET_COMFORT,
                    PRESET_ECO,
                    PRESET_AUTO,
                    PRESET_FROST,
                ]
            self._supported_features |= ClimateEntityFeature.PRESET_MODE

        @property
        def hvac_modes(self) -> list[HVACMode]:
            """Modes HVAC exposés :
            - ECS (EP 6) : Arrêt, Chauffage
            - Chauffage (EP 1 & EP 2) : Arrêt, Chauffage, Automatique
            """
            if self.is_dhw:
                return [HVACMode.OFF, HVACMode.HEAT]
            return [HVACMode.OFF, HVACMode.HEAT, HVACMode.AUTO]

        @property
        def min_temp(self) -> float:
            """Température minimale ajustable."""
            return 30.0 if self.is_dhw else 7.0

        @property
        def max_temp(self) -> float:
            """Température maximale ajustable."""
            return 65.0 if self.is_dhw else 30.0

        @property
        def target_temperature_step(self) -> float:
            """Incrément de consigne (0.5 °C)."""
            return 0.5

        @property
        def current_temperature(self) -> float | None:
            """Température mesurée, ou None si la passerelle n'en a pas.

            Quand le régulateur ne fournit plus la mesure (bus BSB coupé, sonde en défaut), le firmware
            publie la valeur invalide ZCL 0x8000. Sans ce filtre, la carte afficherait -327,7 °C.
            """
            raw = self._local_temperature
            if raw is None or raw == ZCL_INVALID_TEMPERATURE:
                return None
            return raw / ZCL_TEMP

        @property
        def hvac_mode(self) -> HVACMode | None:
            """Retourne le mode HVAC actif."""
            mode = self._system_mode
            if mode is None:
                return None
            if mode == SystemMode.Off:
                return HVACMode.OFF
            if not self.is_dhw and mode == SystemMode.Auto:
                return HVACMode.AUTO
            return HVACMode.HEAT

        @property
        def preset_mode(self) -> str | None:
            """Retourne le préréglage actif dérivé du régime de la PAC."""
            mode = self._system_mode
            if mode is None:
                return None
            if mode == SystemMode.Off:
                return PRESET_FROST
            if mode == SystemMode.Sleep:
                return PRESET_ECO
            if not self.is_dhw and mode == SystemMode.Auto:
                return PRESET_AUTO
            if mode in (SystemMode.Heat, SystemMode.Emergency_Heating):
                return PRESET_COMFORT
            return PRESET_COMFORT if self.is_dhw else PRESET_AUTO

        @property
        def target_temperature(self) -> float | None:
            """Retourne la consigne de température courante selon le régime."""
            if self.hvac_mode in (HVACMode.HEAT, HVACMode.AUTO):
                if self.preset_mode == PRESET_ECO:
                    temp = self._unoccupied_heating_setpoint
                else:
                    temp = self._occupied_heating_setpoint
                if temp is not None:
                    return round(temp / ZCL_TEMP, 1)
            return None

        async def async_set_hvac_mode(self, hvac_mode: HVACMode) -> None:
            """Définit le mode principal CVC."""
            if hvac_mode not in self.hvac_modes:
                return

            if hvac_mode == HVACMode.OFF:
                zcl_mode = SystemMode.Off
            elif hvac_mode == HVACMode.AUTO and not self.is_dhw:
                zcl_mode = SystemMode.Auto
            else:  # HVACMode.HEAT
                zcl_mode = SystemMode.Heat

            await write_attributes_safe(
                self._cluster,
                {ThermostatCluster.AttributeDefs.system_mode.name: zcl_mode},
            )
            self._cluster.update_attribute(
                ThermostatCluster.AttributeDefs.system_mode.id,
                zcl_mode,
            )
            self.maybe_emit_state_changed_event()

        async def async_set_preset_mode(self, preset_mode: str) -> None:
            """Définit le préréglage (Confort, Éco, Auto, Hors-gel)."""
            if preset_mode in (PRESET_COMFORT, "comfort"):
                zcl_mode = SystemMode.Heat
            elif preset_mode in (PRESET_ECO, "eco"):
                zcl_mode = SystemMode.Sleep
            elif not self.is_dhw and preset_mode in (PRESET_AUTO, "auto", "home"):
                zcl_mode = SystemMode.Auto
            elif preset_mode in (PRESET_FROST, "frost_protection", "away", "off"):
                zcl_mode = SystemMode.Off
            else:
                return

            await write_attributes_safe(
                self._cluster,
                {ThermostatCluster.AttributeDefs.system_mode.name: zcl_mode},
            )
            self._cluster.update_attribute(
                ThermostatCluster.AttributeDefs.system_mode.id,
                zcl_mode,
            )
            self.maybe_emit_state_changed_event()

        async def async_set_temperature(
            self,
            target_temp_low: float | None = None,
            target_temp_high: float | None = None,
            temperature: float | None = None,
            hvac_mode: HVACMode | None = None,
        ) -> None:
            """Ajuste la température de consigne."""
            if temperature is None:
                return
            if hvac_mode is not None:
                await self.async_set_hvac_mode(hvac_mode)

            temp_zcl = int(round(temperature * ZCL_TEMP))
            is_eco = (self.preset_mode == PRESET_ECO)
            await self._async_set_heating_setpoint(temp_zcl, is_away=is_eco)

            attr_def = (
                ThermostatCluster.AttributeDefs.unoccupied_heating_setpoint
                if is_eco
                else ThermostatCluster.AttributeDefs.occupied_heating_setpoint
            )
            self._cluster.update_attribute(attr_def.id, temp_zcl)
            self.maybe_emit_state_changed_event()

except Exception as err:
    _LOGGER.warning("Impossible d'enregistrer l'entité AtlanticSiemensThermostat: %s", err)
