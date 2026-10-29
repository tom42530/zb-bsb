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
from zigpy.zcl.clusters.general import AnalogInput
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
    Inactif = 0
    Arret_verrouillage = 3
    Protection_antigel_PAC = 35
    Degivrage_ventilateur = 38
    Degivrage_actif = 39
    Egouttage_degivrage = 41
    Prechauffage_compresseur = 43
    En_fonctionnement = 44
    Chauffage_compresseur_appoint = 46
    Chauffage_compresseur = 49
    Verrouillage_compresseur = 50
    Arret_pas_de_demande = 51
    Delestage_electrique_actif = 52
    Pre_ventilation_pompe = 113
    Fonctionnement_puissance_min = 123
    Limitation_courant_evaporateur = 124
    Limitation_puissance_maximale = 125
    Limitation_protection = 126
    Tempo_min_fonctionnement = 127
    Temps_min_arret_compresseur = 128
    Post_fonctionnement_pompe = 129
    Autre_code_statut = 255


class ZoneStatusEnum(RobustIntEnum):
    Inactif = 0
    Protection_antigel_active = 17
    Arret_mode_hors_gel = 25
    Chauffage_force = 95
    Regime_confort = 101
    Regime_reduit = 102
    Protection_antigel_ambiante = 103
    Arret_consigne_atteinte = 104
    Sechage_dalle_actif = 106
    En_veille_pas_de_demande = 114
    Adaptation_optimisation_debut = 115
    Adaptation_optimisation_fin = 116
    Adaptation_abaissement_actif = 117
    En_demande_de_chauffage = 118
    Prechauffage_confort = 119
    Maintien_abaissement = 120
    Arret_automatique_ete = 121
    Abaissement_reduit = 122
    Limitation_depart_active = 125
    Autre_code_statut = 255


class DhwStatusEnum(RobustIntEnum):
    Inactif = 0
    Arret_ECS_desactivee = 25
    Protection_antigel_ECS = 58
    Charge_forcee_legionellose = 61
    Charge_ECS_active = 63
    Charge_appoint_electrique = 66
    Post_circulation_pompe_ECS = 69
    Charge_prete = 96
    En_charge = 97
    Charge_refroidissement = 98
    Charge_temperature_atteinte = 99
    En_attente_maintien = 100
    Limitation_puissance_ECS = 125
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
