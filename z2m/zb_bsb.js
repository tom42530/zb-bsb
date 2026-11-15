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

// Tables de codes statut Siemens (ENUM8006 / ENUM8000 / ENUM8003), identiques à celles du firmware
const HP_STATUS_FR = {
    0: '---',
    2: 'Défaut',
    10: 'Verrouillé',
    17: 'Post-circulation active',
    23: 'Hors-gel installation actif',
    24: 'Protection hors-gel active',
    25: 'Arrêt',
    26: 'Mode secours',
    27: 'Verrouillé, externe',
    29: 'Haute pression en fonctionnement PAC',
    30: 'Contrôleur de débit source',
    31: 'Pressostat source',
    32: 'Limitation gaz chauds compresseur 1',
    33: 'Limitation gaz chauds compresseur 2',
    34: 'Limitation temp. de coupure max',
    35: 'Temps d’arrêt min. compresseur actif',
    36: 'Compensation excédent de chaleur',
    37: 'Temps de limitation actif',
    38: 'Temps de marche min. compresseur actif',
    39: 'Compensation déficit de chaleur',
    40: 'Limitation écart condenseur max',
    41: 'Limitation écart condenseur min',
    42: 'Limitation écart évaporateur max',
    43: 'Limitation écart évaporateur min',
    44: 'Compresseur et appoint électrique en marche',
    45: 'Compresseurs 1 et 2 en marche',
    46: 'Compresseur 1 en marche',
    47: 'Compresseur 2 en marche',
    48: 'Hors-gel pompe à chaleur',
    49: 'Pré-circulation active',
    50: 'Libérée, compresseur prêt',
    51: 'Pas de demande',
    125: 'Dégivrage actif',
    126: 'Égouttage',
    127: 'Rafraîchissement actif',
    128: 'Rafraîchissement passif',
    129: 'Refroidissement évaporateur',
    130: 'Préchauffage pour dégivrage',
    137: 'Régime de chauffe',
    139: 'Limitation temp. de coupure min',
    145: 'Limitation temp. ext. max rafraîchissement',
    176: 'Verrouillé, température extérieure',
    180: 'Triphasé asymétrique',
    181: 'Basse pression',
    182: 'Surcharge ventilateur',
    183: 'Surcharge compresseur 1',
    184: 'Surcharge compresseur 2',
    185: 'Surcharge pompe source',
    186: 'Contrôleur de débit consommateurs',
    187: 'Limite d’utilisation temp. ext. min',
    188: 'Limite d’utilisation temp. ext. max',
    189: 'Limitation temp. source min (eau)',
    190: 'Limitation temp. source min (eau glycolée)',
    191: 'Limitation temp. source max',
    192: 'Dégivrage forcé par compresseur',
    193: 'Dégivrage forcé par ventilateur',
    194: 'Dégivrage par compresseur',
    195: 'Dégivrage par ventilateur',
    196: 'Limitation temp. source min rafraîchissement',
    197: 'Appoint électrique en marche',
    198: 'Verrouillé, mode éco',
    207: 'Temps de marche min. compresseur, rafraîchissement',
    208: 'Compresseurs 1 et 2 en marche, rafraîchissement',
    209: 'Compresseur 1 en marche, rafraîchissement',
    210: 'Compresseur 2 en marche, rafraîchissement',
    235: 'Pression d’eau trop basse',
    242: 'Déstratification active',
    246: 'Sous-tension réseau',
    254: 'Récupération fluide frigorigène, manuel',
    256: 'Récupération fluide frigorigène',
    257: 'Temporisation démarrage dégivrage',
    258: 'Compresseur verrouillé',
    259: 'Verrouillé, temp. source max',
    260: 'Verrouillé, temp. source min',
    261: 'Verrouillé, temp. retour max',
    262: 'Verrouillé, temp. retour min',
    263: 'Verrouillé, temp. départ max',
    264: 'Verrouillé, temp. départ min',
    265: 'Verrouillé, temp. condenseur max',
    266: 'Verrouillé, temp. évaporateur min',
    267: 'Verrouillé, temp. gaz chauds max',
    268: 'Limitation temp. évaporation min',
    269: 'Limitation temp. condensation max',
    270: 'Limitation temp. évaporation max',
};

// Les codes 22 à 25 (hors-gel / arrêt) diffèrent selon les générations de régulateurs
const ZONE_STATUS_FR = {
    0: '---',
    3: 'Thermostat limiteur déclenché',
    4: 'Régime manuel actif',
    17: 'Post-circulation active',
    22: 'Hors-gel installation actif',
    23: 'Protection hors-gel active',
    24: 'Arrêt',
    25: 'Arrêt',
    56: 'Protection surchauffe active',
    101: 'Hors-gel ambiance actif',
    102: 'Séchage contrôlé de dalle actif',
    103: 'Restreint, protection générateur',
    104: 'Restreint, priorité ECS',
    105: 'Restreint, ballon tampon',
    106: 'Régime de chauffe restreint',
    107: 'Évacuation forcée, ballon tampon',
    108: 'Évacuation forcée, ECS',
    109: 'Évacuation forcée, générateur',
    110: 'Évacuation forcée',
    111: 'Optimisation enclenchement + chauffe accélérée',
    112: 'Optimisation à l’enclenchement',
    113: 'Mise en température accélérée',
    114: 'Chauffage confort',
    115: 'Optimisation à la coupure',
    116: 'Chauffage réduit',
    117: 'Hors-gel départ actif',
    118: 'Régime d’été',
    119: 'Éco journalier actif',
    120: 'Abaissement réduit',
    121: 'Abaissement hors-gel',
    122: 'Limitation température ambiante',
    248: 'Marche continue de la pompe',
};

const DHW_STATUS_FR = {
    0: '---',
    3: 'Thermostat limiteur déclenché',
    4: 'Régime manuel actif',
    17: 'Post-circulation active',
    24: 'Protection hors-gel active',
    25: 'Arrêt',
    53: 'Refroidissement actif',
    66: 'Charge par résistance électrique',
    67: 'Charge forcée active',
    69: 'Charge active',
    70: 'Charge terminée, temp. max ballon',
    71: 'Charge terminée, temp. max de charge',
    75: 'Charge terminée',
    77: 'Refroidissement par collecteur',
    78: 'Refroidissement par générateur/circuits',
    79: 'Protection de décharge active',
    80: 'Limitation durée de charge active',
    81: 'Charge verrouillée',
    82: 'Verrouillage de charge actif',
    83: 'Charge forcée, temp. max ballon',
    84: 'Charge forcée, temp. max de charge',
    85: 'Charge forcée, consigne légionelles',
    86: 'Charge forcée, consigne nominale',
    87: 'Charge électrique, consigne légionelles',
    88: 'Charge électrique, consigne nominale',
    89: 'Charge électrique, consigne réduite',
    90: 'Charge électrique, consigne hors-gel',
    91: 'Résistance électrique libérée',
    92: 'Push, consigne légionelles',
    93: 'Push, consigne nominale',
    94: 'Push actif',
    95: 'En charge, consigne légionelles',
    96: 'En charge, consigne nominale',
    97: 'En charge, consigne réduite',
    98: 'Charge terminée, temp. légionelles',
    99: 'Charge terminée, temp. nominale',
    100: 'Charge terminée, temp. réduite',
    199: 'Soutirage',
    200: 'Prêt',
    201: 'Charge de maintien',
};

const fzLocal = {
    bsb_analog_inputs: {
        cluster: 'genAnalogInput',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.presentValue === undefined) return;
            const val = Number(msg.data.presentValue);
            const ep = msg.endpoint.ID;
            // NaN = grandeur que le régulateur ne fournit plus (bus BSB coupé, sonde en défaut) :
            // elle est publiée `null` plutôt que de rester sur sa dernière valeur.
            const known = Number.isFinite(val);
            const code = known ? Math.round(val) : null;
            const text = (table) => (known ? table[code] || `Code ${code}` : null);

            if (ep === 1) return {compressor_modulation: code};
            if (ep === 2) return {
                zone2_status_code: code,
                zone2_status_text: text(ZONE_STATUS_FR),
            };
            if (ep === 3) return {
                hp_status_code: code,
                hp_status_text: text(HP_STATUS_FR),
            };
            if (ep === 4) return {
                zone1_status_code: code,
                zone1_status_text: text(ZONE_STATUS_FR),
            };
            if (ep === 5) return {zone2_active_setpoint: known ? Number(val.toFixed(1)) : null};
            if (ep === 6) return {
                dhw_status_code: code,
                dhw_status_text: text(DHW_STATUS_FR),
            };
            if (ep === 7) return {backup_heater_modulation: code};
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
        // Thermostats Zone 1, Zone 2 et Ballon ECS : UN appel par endpoint. Les options de m.thermostat()
        // sont `endpoint` (au singulier), `systemMode`, `runningState: {values}` et `piHeatingDemand: {values}` ;
        // toute autre orthographe est ignorée sans erreur et laisse un thermostat sans mode ni état.
        // SystemMode : off = Hors-gel, auto = programme horaire, heat = Confort, sleep = Réduit (Éco).
        // Le firmware publie PIHeatingDemand en pourcentage (0-100) : pas de conversion depuis 0-255.
        m.thermostat({
            endpoint: 'z1',
            setpoints: {
                values: {
                    occupiedHeatingSetpoint: {min: 7, max: 30, step: 0.5},
                    unoccupiedHeatingSetpoint: {min: 7, max: 30, step: 0.5},
                },
            },
            systemMode: {values: ['off', 'auto', 'heat', 'sleep']},
            runningState: {values: ['idle', 'heat']},
            piHeatingDemand: {values: true, dontMapPIHeatingDemand: true},
        }),
        m.thermostat({
            endpoint: 'z2',
            setpoints: {
                values: {
                    occupiedHeatingSetpoint: {min: 7, max: 30, step: 0.5},
                    unoccupiedHeatingSetpoint: {min: 7, max: 30, step: 0.5},
                },
            },
            systemMode: {values: ['off', 'auto', 'heat', 'sleep']},
            runningState: {values: ['idle', 'heat']},
            piHeatingDemand: {values: true, dontMapPIHeatingDemand: true},
        }),
        // Ballon ECS : off = Arrêt, heat = Marche, sleep = Éco
        m.thermostat({
            endpoint: 'dhw',
            setpoints: {
                values: {
                    occupiedHeatingSetpoint: {min: 30, max: 65, step: 0.5},
                    unoccupiedHeatingSetpoint: {min: 30, max: 65, step: 0.5},
                },
            },
            systemMode: {values: ['off', 'heat', 'sleep']},
            runningState: {values: ['idle', 'heat']},
            piHeatingDemand: {values: true, dontMapPIHeatingDemand: true},
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
            access: 'STATE_GET',
            description: 'État marche/arrêt du compresseur PAC (8400)',
        }),
        m.binary({
            name: 'zone2_pump_running',
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            endpointName: 'z2',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            access: 'STATE_GET',
            description: 'État pompe Zone 2 Q6 (8760)',
        }),
        m.binary({
            // L'entrée binaire de l'EP3 porte le circulateur principal de la PAC (8406) lorsqu'il existe,
            // sinon la pompe du circuit Zone 1 Q2 (8730) : d'où un nom neutre.
            name: 'primary_pump_running',
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            endpointName: 'hyd1',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            access: 'STATE_GET',
            description: 'Circulateur principal PAC (8406), ou pompe Zone 1 Q2 (8730) si la PAC n’a pas de circulateur dédié',
        }),
        m.binary({
            name: 'backup_heater_running',
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            endpointName: 'aux',
            valueOn: ['ON', 1],
            valueOff: ['OFF', 0],
            access: 'STATE_GET',
            description: 'État appoint électrique actif (8402)',
        }),
        // Sorties analogiques R/W (0x000D : Consignes réduites Z1/Z2, Sondes Externes Z1/Z2, Consigne ECS).
        // IMPORTANT : m.numeric attend `endpointNames` (tableau). Avec `endpointName` (singulier, propre à
        // m.binary), l'option est ignorée et TOUTES les écritures partent sur l'endpoint 1, c'est-à-dire sur
        // la consigne réduite de la Zone 1. Les propriétés exposées portent le suffixe de l'endpoint :
        //   reduced_setpoint_z1 / reduced_setpoint_z2, external_room_temp_hyd1 (Zone 1) /
        //   external_room_temp_hyd2 (Zone 2), nominal_setpoint_dhw.
        m.numeric({
            name: 'reduced_setpoint',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointNames: ['z1', 'z2'],
            unit: '°C',
            valueMin: 7,
            valueMax: 30,
            valueStep: 0.5,
            access: 'ALL',
            description: 'Consigne Réduite / Éco de la zone (712 pour z1, 1012 pour z2)',
        }),
        m.numeric({
            name: 'external_room_temp',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointNames: ['hyd1', 'hyd2'],
            unit: '°C',
            valueMin: 0,
            valueMax: 35,
            valueStep: 0.1,
            access: 'ALL',
            description: 'Sonde Température Ambiante Externe (hyd1 = Zone 1, hyd2 = Zone 2 ; 0 = désactivée / retour BSB)',
        }),
        m.numeric({
            name: 'nominal_setpoint',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointNames: ['dhw'],
            unit: '°C',
            valueMin: 30,
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
        // Switch Antenne RF Zigbee Externe U.FL sur Endpoint 7 (OFF par défaut = Céramique interne)
        m.onOff({
            endpointNames: ['aux'],
            powerOnBehavior: false,
            description: 'Activation Antenne Zigbee Externe U.FL (OFF = Céramique interne, ON = U.FL externe)',
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
