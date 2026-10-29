/**
 * Convertisseur externe Zigbee2MQTT pour la passerelle ZB-BSB (Atlantic / Siemens RVS21)
 *
 * Installation dans Zigbee2MQTT :
 *   1. Copiez ce fichier dans le dossier `data/external_converters/zb_bsb.js` de Zigbee2MQTT
 *   2. Redémarrez Zigbee2MQTT.
 *
 * Ce convertisseur supporte l'architecture multi-endpoints dynamique de ZB-BSB,
 * l'injection d'une sonde de température externe par zone (Zone 1 et Zone 2),
 * et traduit automatiquement les codes statut numériques (8000, 8001, 8003, 8006)
 * en états texte explicites en français.
 */

const m = require('zigbee-herdsman-converters/lib/modernExtend');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;
const ea = exposes.access;

const HP_STATUS_FR = {
    3: 'Arrêt verrouillage',
    35: 'Protection antigel PAC',
    38: 'Dégivrage ventilateur',
    39: 'Dégivrage actif',
    41: 'Égouttage dégivrage',
    43: 'Préchauffage compresseur',
    44: 'En fonctionnement (Chauffage/ECS)',
    46: 'Chauffage compresseur + appoint',
    49: 'Chauffage compresseur',
    50: 'Verrouillage temporaire compresseur',
    51: 'Arrêt / Pas de demande de chaleur',
    52: 'Délestage électrique actif',
    113: 'Pré-ventilation pompe',
    123: 'Fonctionnement puissance min.',
    124: 'Limitation courant évaporateur',
    125: 'Limitation puissance maximale',
    126: 'Limitation / Protection',
    127: 'Tempo min. fonctionnement',
    128: 'Temps min. arrêt compresseur actif',
    129: 'Post-fonctionnement pompe',
};

const ZONE_STATUS_FR = {
    17: 'Protection antigel active',
    25: 'Arrêt (Mode Hors-Gel)',
    95: 'Chauffage forcé',
    101: 'Régime de chauffe Confort',
    102: 'Régime de chauffe Réduit',
    103: 'Protection antigel ambiante',
    104: 'Limitation / Arrêt ambiance atteinte',
    106: 'Séchage dalle actif',
    114: 'En veille (Pas de demande)',
    115: 'Adaptation optimisation début',
    116: 'Adaptation optimisation fin',
    117: 'Adaptation / Abaissement actif',
    118: 'En demande de chauffage',
    119: 'Préchauffage confort',
    120: 'Maintien abaissement',
    121: 'Arrêt automatique (Mode Été)',
    122: 'Abaissement réduit',
    125: 'Limitation départ active',
};

const DHW_STATUS_FR = {
    25: 'Arrêt (ECS désactivée)',
    58: 'Protection antigel ECS',
    61: 'Charge ECS forcée / Légionellose',
    63: 'Charge ECS active (Chauffe en cours)',
    66: 'Charge par appoint électrique',
    69: 'Post-circulation pompe ECS',
    96: 'Charge prête',
    97: 'En charge',
    98: 'Charge refroidissement',
    99: 'Chargé (Prêt - Temp. atteinte)',
    100: 'En attente / Maintien',
    125: 'Limitation puissance ECS',
};

const fzLocal = {
    bsb_analog_inputs: {
        cluster: 'genAnalogInput',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.presentValue === undefined) return;
            const val = Number(msg.data.presentValue);
            const ep = msg.endpoint.ID;
            const code = Math.round(val);

            if (ep === 1) return {compressor_modulation: Math.round(val)};
            if (ep === 2) return {
                zone2_status_code: code,
                zone2_status_text: ZONE_STATUS_FR[code] || `Code ${code}`,
            };
            if (ep === 3) return {
                hp_status_code: code,
                hp_status_text: HP_STATUS_FR[code] || `Code ${code}`,
            };
            if (ep === 4) return {
                zone1_status_code: code,
                zone1_status_text: ZONE_STATUS_FR[code] || `Code ${code}`,
            };
            if (ep === 5) return {zone2_active_setpoint: Number(val.toFixed(1))};
            if (ep === 6) return {
                dhw_status_code: code,
                dhw_status_text: DHW_STATUS_FR[code] || `Code ${code}`,
            };
            if (ep === 7) return {backup_heater_modulation: Math.round(val)};
        },
    },
};

const definition = {
    zigbeeModel: ['ZB-BSB'],
    model: 'ZB-BSB',
    vendor: 'Atlantic-Siemens',
    description: 'Passerelle Zigbee 3.0 autonome pour Pompe à Chaleur BSB (Auto-Discovery)',
    fromZigbee: [fzLocal.bsb_analog_inputs],
    extend: [
        m.deviceEndpoints({
            endpoints: {
                z1: 1,
                z2: 2,
                hyd1: 3,
                hyd2: 4,
                flow_z2: 5,
                dhw: 6,
                aux: 7,
            },
        }),
        // Thermostats Zone 1 & Zone 2
        m.thermostat({
            endpointNames: ['z1', 'z2'],
            setpoints: {values: {occupiedHeatingSetpoint: {min: 7, max: 30, step: 0.5}}},
            systemModes: {values: ['off', 'auto', 'heat', 'sleep']},
            runningState: true,
            piHeatingDemand: true,
        }),
        // Mesures de température natives (0x0402)
        m.temperature({endpointNames: ['z1', 'z2', 'hyd1', 'hyd2', 'flow_z2', 'dhw', 'aux']}),
        // Entrées binaires (0x000F : Compresseur, Pompes Z1/Z2, Appoint)
        m.binary({
            name: 'compressor_running',
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            endpointName: 'z1',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            description: 'État marche/arrêt du compresseur PAC (8400)',
        }),
        m.binary({
            name: 'zone2_pump_running',
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            endpointName: 'z2',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            description: 'État pompe Zone 2 Q6 (8760)',
        }),
        m.binary({
            name: 'zone1_pump_running',
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            endpointName: 'hyd1',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            description: 'État pompe Zone 1 Q2 (8730)',
        }),
        m.binary({
            name: 'backup_heater_running',
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            endpointName: 'aux',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            description: 'État appoint électrique actif (8402)',
        }),
        // Sorties analogiques R/W (0x000D : Consignes réduites Z1/Z2, Sondes Externes Z1/Z2, Consigne ECS)
        m.numeric({
            name: 'zone1_reduced_setpoint',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointName: 'z1',
            unit: '°C',
            valueMin: 7,
            valueMax: 30,
            valueStep: 0.5,
            access: 'ALL',
            description: 'Consigne Réduite / Éco Zone 1 (712)',
        }),
        m.numeric({
            name: 'zone2_reduced_setpoint',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointName: 'z2',
            unit: '°C',
            valueMin: 7,
            valueMax: 30,
            valueStep: 0.5,
            access: 'ALL',
            description: 'Consigne Réduite / Éco Zone 2 (1012)',
        }),
        m.numeric({
            name: 'zone1_external_room_temp',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointName: 'hyd1',
            unit: '°C',
            valueMin: 0,
            valueMax: 35,
            valueStep: 0.1,
            access: 'ALL',
            description: 'Sonde Température Ambiante Externe Zone 1 (0 = désactivée / retour BSB)',
        }),
        m.numeric({
            name: 'zone2_external_room_temp',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointName: 'hyd2',
            unit: '°C',
            valueMin: 0,
            valueMax: 35,
            valueStep: 0.1,
            access: 'ALL',
            description: 'Sonde Température Ambiante Externe Zone 2 (0 = désactivée / retour BSB)',
        }),
        m.numeric({
            name: 'dhw_nominal_setpoint',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointName: 'dhw',
            unit: '°C',
            valueMin: 35,
            valueMax: 65,
            valueStep: 0.5,
            access: 'ALL',
            description: 'Consigne Nominale Ballon ECS (1610)',
        }),
        // Switch Marche/Arrêt ECS sur Endpoint 6
        m.onOff({
            endpointNames: ['dhw'],
            powerOnBehavior: false,
            description: 'Activation production Eau Chaude Sanitaire (1600)',
        }),
    ],
    exposes: [
        e.numeric('compressor_modulation', ea.STATE).withUnit('%').withDescription('Modulation du compresseur (8413)'),
        e.numeric('backup_heater_modulation', ea.STATE).withUnit('%').withDescription('Modulation appoint électrique (8414)'),
        e.numeric('zone2_active_setpoint', ea.STATE).withUnit('°C').withDescription('Consigne active Zone 2 (8771)'),
        e.text('hp_status_text', ea.STATE).withDescription('État actuel de la PAC (8006)'),
        e.text('zone1_status_text', ea.STATE).withDescription('État actuel Zone 1 (8000)'),
        e.text('zone2_status_text', ea.STATE).withDescription('État actuel Zone 2 (8001)'),
        e.text('dhw_status_text', ea.STATE).withDescription('État actuel Ballon ECS (8003)'),
    ],
};

module.exports = definition;
