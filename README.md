# MTO_JollyBall

### **Hoofdrequirements (R)**

- **R1:** De robot moet zelfstandig kunnen balanceren op een bal ter grootte van een voetbal op een vlakke ondergrond.
    - Drie identieke omni-wielen aangedreven door DC-motoren met bijbehorende motor drivers.
    - IMU (Inertial Measurement Unit) en gyroscoop voor balansdetectie.
    - Een stevig en gebalanceerd frame.
    - Arduino-gebaseerde besturing voor motorregeling.
    - PID-regelaar om de robot stabiel en rechtop te houden.

- **R2:** De robot mag geen bedrade verbinding hebben met externe systemen of de omgeving.
    - Werkt op een interne batterij.
    - Alle verwerking gebeurt intern via een Arduino.

- **R3:** De robot moet minimaal 15 minuten continu kunnen functioneren zonder fouten.
 
---

### **Uitbreidingen (UR)**

- **UR1:** De robot moet op afstand bestuurbaar zijn via een smartphone-app.
    - Besturing via een webpagina op een smartphone.
    - Raspberry Pi fungeert als server.
    - De Raspberry Pi communiceert met de Arduino en de motoren.

- **UR2:** De robot moet zijn positie kunnen behouden en actief corrigeren bij externe verstoringen.
    - Detecteert onverwachte bewegingen door het meten van motoractiviteit (bijvoorbeeld met encoders of stroommetingen).
    - Softwarematig variabel instelbaar nulpunt om afwijkingen te corrigeren.

- **UR3:** De robot moet in staat zijn om een variabel doolhof op te lossen.
    - Doolhof met een knikker als simulatie-element.
    - Robot moet voldoende kunnen kantelen om knikker te laten bewegen.
    - Camera om de lay-out van het doolhof te visualiseren.
    - Camera om de locatie van de knikker te bepalen.
    - Software voor muurherkenning.
    - Software om de knikkerpositie te analyseren.
    - Software om muren om te vormen naar een grafische representatie (graph).
    - Algoritme om de optimale route in de graph te berekenen.

- **UR4:** De robot moet obstakels kunnen detecteren en vermijden (collision detection).
    - 2D Lidar voor omgevingsdetectie.

### Componentenlijst
#### **1. Mechanische componenten**

- **Omni-wielen (3x)** – Voor 360° beweging en balans
- **DC-motoren (3x)** – Aandrijving van de omni-wielen
- **Motor drivers (3x)** – Aansturing van de DC-motoren
- **Frame** – Gebalanceerd en stevig, geschikt voor montage van alle componenten
- **Bal (voetbalgrootte, stevige grip)** – Ondergrond voor de robot

#### **2. Elektronische componenten**

- **Arduino (1x)** – Hoofdcontroller voor motorsturing en balansregeling
- **IMU (Inertial Measurement Unit) (1x)** – Voor balansdetectie (incl. gyroscoop en versnellingsmeter)
- **Encoders of stroommeting op motoren (optioneel)** – Voor positiedetectie en zelfcorrectie
- **Interne batterij (1x)** – Draadloze voeding voor de robot
- **Bekabeling en connectoren** – Voor de verbinding van alle elektronische componenten
- **Raspberry Pi (1x)** – Voor serverfunctionaliteit en geavanceerde verwerking
- **Wi-Fi-module (ingebouwd in Raspberry Pi)** – Voor draadloze communicatie
- **Softwarematige webinterface** – Voor smartphonebesturing

#### **3. Sensoren en beeldverwerking**

- **2D Lidar (1x)** – Voor obstakeldetectie en navigatie
- **Camera (1x)** – Voor doolhofvisualisatie en knikkertracking
