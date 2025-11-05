# Bitcraze Flow Breakout - ROS Package

ROS package pour interfacer la carte **Bitcraze Flow Deck Breakout** avec ROS. Cette carte intègre deux capteurs essentiels pour la navigation de drones et robots :

- **PMW3901** : Capteur de flux optique pour mesurer le déplacement en X/Y
- **VL53L0X** : Capteur ToF (Time-of-Flight) pour mesurer la distance

## 📋 Table des matières

- [Prérequis](#prérequis)
- [Installation](#installation)
- [Câblage](#câblage)
- [Utilisation](#utilisation)
- [Paramètres ROS](#paramètres-ros)
- [Messages ROS](#messages-ros)
- [Exemples](#exemples)
- [Dépannage](#dépannage)

## 🔧 Prérequis

### Matériel
- Raspberry Pi (ou autre carte avec GPIO et I2C/SPI)
- Carte Bitcraze Flow Deck Breakout
- Alimentation 3.3V pour la carte

### Logiciels
- ROS (testé avec Kinetic/Melodic/Noetic)
- Bibliothèque bcm2835 (pour accès GPIO Raspberry Pi)
- Droits root pour accès aux périphériques I2C/SPI

### Installation de bcm2835

```bash
# Télécharger et installer la bibliothèque bcm2835
cd /tmp
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz
tar zxvf bcm2835-1.71.tar.gz
cd bcm2835-1.71
./configure
make
sudo make check
sudo make install
```

## 📦 Installation

```bash
# Cloner le package dans votre workspace catkin
cd ~/catkin_ws/src
git clone <repository_url> bitcraze_flow_breakout

# Compiler le package
cd ~/catkin_ws
catkin_make

# Sourcer le workspace
source devel/setup.bash
```

## 🔌 Câblage

### PMW3901 (Capteur de flux optique - SPI)

| Pin Capteur | Pin Raspberry Pi | Fonction |
|-------------|------------------|----------|
| GND         | GND (Pin 6)      | Ground   |
| 3.3V        | 3.3V (Pin 1)     | Alimentation |
| CS          | GPIO 18 (Pin 12) | Chip Select |
| CLK         | SCLK (Pin 23)    | SPI Clock |
| MOSI        | MOSI (Pin 19)    | SPI Data Out |
| MISO        | MISO (Pin 21)    | SPI Data In |

### VL53L0X (Capteur ToF - I2C)

| Pin Capteur | Pin Raspberry Pi | Fonction |
|-------------|------------------|----------|
| GND         | GND              | Ground   |
| VCC         | 3.3V             | Alimentation |
| SDA         | SDA (GPIO 2)     | I2C Data |
| SCL         | SCL (GPIO 3)     | I2C Clock |

**Important** : Activez I2C et SPI sur votre Raspberry Pi :
```bash
sudo raspi-config
# Interfacing Options > I2C > Enable
# Interfacing Options > SPI > Enable
```

## 🚀 Utilisation

### Lancer tous les nœuds

```bash
# Avec les paramètres par défaut
sudo -E roslaunch bitcraze_flow_breakout bitcraze.launch

# Avec des paramètres personnalisés
sudo -E roslaunch bitcraze_flow_breakout bitcraze.launch \
    flow_rate:=20.0 \
    tof_rate:=15.0 \
    tof_long_range:=true
```

**Note** : `sudo -E` est nécessaire pour accéder aux périphériques SPI/I2C tout en conservant les variables d'environnement ROS.

### Lancer les nœuds individuellement

```bash
# Nœud du capteur de flux optique
sudo -E rosrun bitcraze_flow_breakout flow_sensor_node

# Nœud du capteur ToF
sudo -E rosrun bitcraze_flow_breakout tof_node
```

## ⚙️ Paramètres ROS

### flow_sensor_node

| Paramètre   | Type   | Défaut        | Description |
|-------------|--------|---------------|-------------|
| `~rate`     | double | 10.0          | Fréquence de publication (Hz) |
| `~topic`    | string | "flow_sensor" | Nom du topic de publication |
| `~frame_id` | string | "flow_sensor" | Frame ID pour le header |
| `~spi_cs_pin` | int  | 1             | Pin CS pour SPI (BCM numbering) |

### tof_node

| Paramètre      | Type   | Défaut | Description |
|----------------|--------|--------|-------------|
| `~rate`        | double | 10.0   | Fréquence de publication (Hz) |
| `~topic`       | string | "tof"  | Nom du topic de publication |
| `~frame_id`    | string | "tof"  | Frame ID pour le header |
| `~timeout_ms`  | int    | 200    | Timeout pour les mesures (ms) |
| `~long_range`  | bool   | false  | Activer le mode longue portée (max ~2m) |
| `~high_speed`  | bool   | false  | Mode haute vitesse (20ms/mesure, moins précis) |
| `~high_accuracy` | bool | false  | Mode haute précision (200ms/mesure) |

**Note** : `high_speed` et `high_accuracy` sont mutuellement exclusifs.

## 📨 Messages ROS

### flow_sensor.msg

```
std_msgs/Header header
float64 deltaX  # Déplacement en X (unités capteur)
float64 deltaY  # Déplacement en Y (unités capteur)
```

- `deltaX`, `deltaY` : Valeurs brutes du capteur PMW3901 (int16 converti en float64)
- Valeurs positives/négatives indiquent la direction du mouvement
- L'échelle dépend de la hauteur et de la surface observée

### tof.msg

```
std_msgs/Header header
float64 distance  # Distance en mètres
bool valid        # True si la mesure est valide
```

- `distance` : Distance mesurée en mètres (0.05m à 1.2m typique, jusqu'à 2m en mode longue portée)
- `valid` : False en cas d'erreur I2C ou de mesure invalide

## 📊 Exemples

### Visualiser les données dans ROS

```bash
# Afficher les données de flux
rostopic echo /flow_sensor

# Afficher les données de distance
rostopic echo /tof

# Vérifier la fréquence de publication
rostopic hz /flow_sensor
rostopic hz /tof
```

### Exemple de nœud Python consommateur

```python
#!/usr/bin/env python
import rospy
from bitcraze_flow_breakout.msg import flow_sensor, tof

def flow_callback(msg):
    rospy.loginfo("Flow: deltaX=%.2f, deltaY=%.2f", msg.deltaX, msg.deltaY)

def tof_callback(msg):
    if msg.valid:
        rospy.loginfo("Distance: %.3f m", msg.distance)
    else:
        rospy.logwarn("Invalid ToF measurement")

rospy.init_node('sensor_reader')
rospy.Subscriber('/flow_sensor', flow_sensor, flow_callback)
rospy.Subscriber('/tof', tof, tof_callback)
rospy.spin()
```

### Fichier launch personnalisé

```xml
<launch>
  <!-- Flow sensor avec fréquence augmentée -->
  <node name="flow_sensor" pkg="bitcraze_flow_breakout" type="flow_sensor_node">
    <param name="rate" value="20.0"/>
    <param name="frame_id" value="base_link"/>
  </node>

  <!-- ToF en mode longue portée -->
  <node name="tof" pkg="bitcraze_flow_breakout" type="tof_node">
    <param name="rate" value="15.0"/>
    <param name="long_range" value="true"/>
    <param name="frame_id" value="base_link"/>
  </node>
</launch>
```

## 🔍 Dépannage

### Erreur "bcm2835_init failed"

**Solution** :
- Vérifiez que vous exécutez avec `sudo`
- Vérifiez que la bibliothèque bcm2835 est installée

### Erreur "Failed to initialize PMW3901 sensor"

**Solutions** :
- Vérifiez le câblage SPI
- Assurez-vous que SPI est activé : `ls /dev/spidev*`
- Vérifiez l'alimentation 3.3V de la carte
- Testez avec `gpio readall` (paquet wiringpi)

### Erreur "Error initializing VL53L0X sensor"

**Solutions** :
- Vérifiez le câblage I2C
- Assurez-vous que I2C est activé : `i2cdetect -y 1` (devrait montrer device à 0x29)
- Vérifiez que personne d'autre n'utilise le bus I2C

### Les mesures semblent erronées

**Flow sensor** :
- Le flux optique nécessite un éclairage suffisant
- Les surfaces lisses/uniformes donnent de mauvais résultats
- La hauteur affecte l'échelle des mesures

**ToF sensor** :
- Surfaces réfléchissantes ou très sombres peuvent donner de mauvaises mesures
- Vérifiez le flag `valid` dans le message
- Utilisez `long_range` pour des distances > 1.2m
- En cas d'erreurs fréquentes, vérifiez les logs avec `ROS_WARN_THROTTLE`

### Permissions I2C/SPI sans sudo

Si vous voulez éviter d'utiliser sudo :

```bash
# Ajouter votre utilisateur aux groupes appropriés
sudo usermod -a -G i2c,spi,gpio $USER

# Créer des règles udev
sudo nano /etc/udev/rules.d/99-spi-i2c.rules

# Ajouter :
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"

# Recharger les règles et redémarrer
sudo udevadm control --reload-rules
sudo reboot
```

## 📚 Références

- [Bitcraze Flow Deck](https://www.bitcraze.io/products/flow-deck-v2/)
- [PMW3901 Datasheet](https://www.pixart.com/products-detail/10/PMW3901MB-TXQT)
- [VL53L0X Datasheet](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html)
- [BCM2835 Library](http://www.airspayce.com/mikem/bcm2835/)

## 📝 Licence

TODO - À définir

## 👥 Contributeurs

- Clément (maintainer)

## 🐛 Signaler des bugs

Pour signaler des bugs ou demander des fonctionnalités, veuillez ouvrir une issue sur le dépôt GitHub.
