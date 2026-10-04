## Digital Pins

| Pin | Fonction |
|---|---|
| D0 | Record : enregistre les 10 dernières secondes |
| D1 | Crop : valide la sélection faite avec le Trill Flex |
| D2 | Drone : onde sine |
| D3 | Drone : onde triangle |
| D4 | Drone : onde square |
| D5 | Drone : onde sawtooth |
| D8 | Drone : marche / arrêt (désactivé au démarrage) |

## Analog Pins

| Pin | Fonction | Plage |
|---|---|---|
| A0 | Filtre : fréquence de coupure | 100 à 1000 Hz |
| A1 | Filtre : résonance (Q) | 0,5 à 10 |
| A2 | Volume principal | 0 à 1 |
| A3 | Volume du drone (indépendant du volume principal) | 0 à 1 |
| A4 | Vitesse de lecture du sample | 0,25x à 3x |
| A5 | Choix de la gamme du drone | 6 positions : 0 = mapping continu, 1 à 5 = gammes |

## Bus I2C 1 (via Trill Hub)

| Signal | Pin Bela |
|---|---|
| SDA | P9.26 |
| SCL | P9.24 |
| VCC | 3,3 V |
| GND | GND |

| Adresse | Appareil | Branchement |
|---|---|---|
| 0x38 | Trill Ring (scratch) | Port Qwiic / JST PH 4 broches |
| 0x48 | Trill Flex (crop) | Port Qwiic / JST PH 4 broches |
| 0x29 | VL53L1X (distance) | Pastilles libres du Hub, ou directement sur les pins I2C |

## Audio & alimentation

| Élément | Branchement |
|---|---|
| Micro MAX9814 : VCC | 3,3 V |
| Micro MAX9814 : GND | GND |
| Micro MAX9814 : GAIN | 3,3 V (gain de 40 dB) |
| Micro MAX9814 : OUT | Condensateur 10 µF, puis broche centrale (Left, canal 0) du connecteur audio IN |
| Haut-parleur 4 Ω, 3 W | Connecteur Speaker (2 broches). La polarité n'est pas critique |
| Alimentation | Batterie 5 V via un câble USB-A vers jack 5,5 x 2,1 mm, centre positif |
| USB | Laptop, optionnel (programmation et console) |