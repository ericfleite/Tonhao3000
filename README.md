# Controle de dois motores com Arduino UNO, L298N e HC-05

Português — Versão: 1.0 e 1.1
Última atualização: 15/11/2025

Este README descreve o projeto que controla 2 motores DC usando um Arduino Uno, um driver de motor L298N e um módulo Bluetooth HC-05. Inclui: mapeamento de pinos, alimentação, cuidados elétricos, comandos via Bluetooth, código (pronto para usar) e dicas de resolução de problemas.

---

## O que este projeto faz

Recebe comandos via Bluetooth (HC-05) e aciona dois motores por meio do L298N. Comandos suportados (por padrão no app Arduino Bluetooth Control):

* `F` — andar para frente (ambos motores para frente)
* `B` — ré (ambos motores para trás)
* `L` — virar para a esquerda (Motor1 frente, Motor2 parado)
* `R` — virar para a direita (Motor2 frente, Motor1 parado)
* `I` — virar para a esquerda (ré) (Motor1 trás, Motor2 parado)
* `J` — virar para a direita (ré) (Motor2 trás, Motor1 parado)
* `S` — parar (ambos motores parados)

---

## Arquivos

* `README.md` 
* `robo_V1.0Eric.ino` - Versão sem usar a biblioteca SoftwareSerial.h
* `robo_V1.1Eric.ino` - Versão usando a biblioteca SoftwareSerial.h

---

## Código (robo_V1.0Eric.ino)

```cpp
#include <SoftwareSerial.h>

char buf;

class DCMotor {  
  int spd = 255, pin1, pin2;
  
public:  
    void Pinout(int in1, int in2){
      pin1 = in1;
      pin2 = in2;
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
    }

    void Speed(int in1){
      spd = in1;
    }    

    void Forward(){
      analogWrite(pin1, spd);
      digitalWrite(pin2, LOW);
    }

    void Backward(){
      digitalWrite(pin1, LOW);
      analogWrite(pin2, spd);
    }

    void Stop(){
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    }
};

DCMotor Motor1, Motor2;

void verificaComandoCelular(){
  int watchDogCounter = 0;

  while(BT.available() > 0){
    buf = BT.read();
    Serial.println(buf);  

    // Frente
    if (buf == 'F'){
      Motor1.Forward();
      Motor2.Forward();
    }

    // Ré no kibe
    if (buf == 'B'){
      Motor1.Backward();
      Motor2.Backward();
    }

    // Esquerda frente
    if (buf == 'L'){
      Motor1.Forward();
      Motor2.Stop();
    }

    // Direita frente
    if (buf == 'R'){
      Motor2.Forward();
      Motor1.Stop();
    }

    // Esquerda trás 
    if (buf == 'I'){
      Motor1.Backward();
      Motor2.Stop();
    }

    // Direita trás 
    if (buf == 'J'){
      Motor2.Backward();
      Motor1.Stop();
    }

    // Parar
    if (buf == 'S'){
      Motor1.Stop();
      Motor2.Stop();
    }

    delay(100);
    watchDogCounter++;
    if (watchDogCounter > 500) break;
  }
}

void setup() {
  Serial.begin(9600);

  Motor1.Pinout(5,6);
  Motor2.Pinout(9,10);

  delay(1000);
}

void loop() {
  Motor1.Speed(255);
  Motor2.Speed(255);
  verificaComandoCelular();
}

```

> O sketch usa 5/6 e 9/10 para os dois motores; 2/3 para SoftwareSerial (HC-05).

---

## Código (robo_V1.1Eric.ino)

```cpp
#include <SoftwareSerial.h>

SoftwareSerial BT(2, 3); // RX, TX

char buf;

class DCMotor {  
  int spd = 255, pin1, pin2;
  
public:  
    void Pinout(int in1, int in2){
      pin1 = in1;
      pin2 = in2;
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
    }

    void Speed(int in1){
      spd = in1;
    }    

    void Forward(){
      analogWrite(pin1, spd);
      digitalWrite(pin2, LOW);
    }

    void Backward(){
      digitalWrite(pin1, LOW);
      analogWrite(pin2, spd);
    }

    void Stop(){
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    }
};

DCMotor Motor1, Motor2;

void verificaComandoCelular(){
  int watchDogCounter = 0;

  while(BT.available() > 0){
    buf = BT.read();
    Serial.println(buf);  

    // Frente
    if (buf == 'F'){
      Motor1.Forward();
      Motor2.Forward();
    }

    // Ré no kibe
    if (buf == 'B'){
      Motor1.Backward();
      Motor2.Backward();
    }

    // Esquerda frente
    if (buf == 'L'){
      Motor1.Forward();
      Motor2.Stop();
    }

    // Direita frente
    if (buf == 'R'){
      Motor2.Forward();
      Motor1.Stop();
    }

    // Esquerda trás 
    if (buf == 'I'){
      Motor1.Backward();
      Motor2.Stop();
    }

    // Direita trás 
    if (buf == 'J'){
      Motor2.Backward();
      Motor1.Stop();
    }

    // Parar
    if (buf == 'S'){
      Motor1.Stop();
      Motor2.Stop();
    }

    delay(100);
    watchDogCounter++;
    if (watchDogCounter > 500) break;
  }
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  Motor1.Pinout(5,6);
  Motor2.Pinout(9,10);

  delay(1000);
}

void loop() {
  Motor1.Speed(255);
  Motor2.Speed(255);
  verificaComandoCelular();
}

```

> O sketch usa 5/6 e 9/10 para os dois motores; 2/3 para SoftwareSerial (HC-05).

---

## Mapeamento de pinos (recomendado — corresponde ao código acima)

### Arduino → L298N

* Arduino D5  → L298N IN1  (Motor1 PIN1 usado com PWM para controlar direção/vel)
* Arduino D6  → L298N IN2
* Arduino D9  → L298N IN3  (Motor2 PIN1 usado com PWM)
* Arduino D10 → L298N IN4

> Obs.: Em muitos módulos L298N há pinos **ENA** (enable A) e **ENB** (enable B). Se seu módulo tem jumpers para ENA/ENB:

* Se os jumpers estiverem **presentes**, os ENA/ENB já estão habilitados (normalmente ligados ao regulador 5V do módulo).
* Se os jumpers **não** estiverem, conecte ENA e ENB ao 5V (ou a um pino PWM se você quiser controlar velocidade por ENx). No nosso código controlamos velocidade via `analogWrite` em INx, portanto deixe ENA/ENB habilitados (jumper 5V).

### Arduino ↔ HC-05 (SoftwareSerial)

* HC-05 TX → Arduino D2 (RX do SoftwareSerial)
* HC-05 RX → Arduino D3 (TX do SoftwareSerial) **COM DIVISOR** (ver observação abaixo)
* HC-05 VCC → 5V
* HC-05 GND → GND

**Importante:** o pino RX do HC-05 espera ~3.3V. Quando você conectar Arduino D3 (5V TTL) ao RX do HC-05, use um divisor de tensão ou conversor de nível (2 resistores formam um divisor: por exemplo 2.2k/1k) para reduzir 5V → ~3.3V e proteger o módulo.

### Alimentação dos motores / L298N

* L298N `+5V` (ou tensão dos seus motores, até limite do L298N/module) → fonte de alimentação do motor (bateria).
* L298N `GND` → comum com Arduino GND e bateria negativa.
* L298N `5V` (se o módulo tiver regulador e o jumper `5V EN` for usado) — dependendo do módulo, esse pino pode fornecer 5V para o Arduino ou ser alimentado por 5V do Arduino; **NÃO** alimente duas fontes conflitantes — entenda o seu módulo.

**Regra de ouro:** NUNCA alimente motores diretamente do pino 5V do Arduino. Use fonte externa para os motores. Ligue os GNDs em comum.

---

## Diagrama elétrico textual

```
[Bateria 5V +] --> L298N VMS (motor +)
[Bateria/GND] ---> L298N GND ----+----> Arduino GND
                                  |
                                  +--> HC-05 GND

Arduino 5V ----> HC-05 VCC (se desejar alimentar HC-05 pelo Arduino)
Arduino D2 <---- HC-05 TX
Arduino D3 ----> HC-05 RX (via divisor de tensão)

Arduino D5 ----> L298N IN1
Arduino D6 ----> L298N IN2
Arduino D9 ----> L298N IN3
Arduino D10 ---> L298N IN4

L298N OUT1/OUT2 -> Motor1
L298N OUT3/OUT4 -> Motor2
```

---

## Cuidados e recomendações elétricas

* Use fonte externa para os motores (bateria adequada à tensão nominal dos motores).
* GND comum obrigatório (Arduino GND, L298N GND e bateria negativa).
* Proteção do HC-05 RX: use um divisor de tensão.
* O L298N aquece — evite correntes altas; utilize dissipador/ventilação. O L298N tem perda de tensão significativa; se motores forem de alto consumo, considere driver moderno (ex.: VNH2SP30, BTS7960, DRV8871, etc.).
* Verifique a corrente dos motores: se exceder o limite do L298N (≈2 A contínuo por canal em condições ideais) o driver pode danificar.
* Se usar a saída 5V do módulo L298N para alimentar o Arduino, certifique-se que a fonte motor consegue fornecer a corrente.

---

## Debug e Solução de Problemas

* Nada acontece:

  * Verifique GND comum.
  * Verifique se HC-05 emparelhou com o telefone (PIN padrão 1234/0000).
  * Use Monitor Serial para ver os caracteres recebidos (o código atual também imprime `Serial.println(buf)`).
* Motor gira somente em uma direção/sempre parado:

  * Confirme ligações INx → Arduino.
  * Confirme jumpers ENA/ENB se necessários.
* HC-05 não responde:

  * Confirme divisor no RX do HC-05 (Arduino → HC-05).
  * Teste comunicação com `BT.println("teste");` e monitore com um app serial Bluetooth.
* Ruídos/Reset:

  * Adicione capacitores de desacoplamento (100µF) na alimentação do motor.
  * Use cabos curtos e bem isolados.

---

## Melhorias sugeridas

* Implementar leitura de watchdog por tempo (para parar o robô se o sinal Bluetooth cair).
* Usar `attachInterrupt` ou millis() para watchdog em vez de `delay`.
* Controlar velocidade via ENA/ENB com PWM (em vez de `analogWrite` em IN pins) para lógica mais clara.
* Implementar smoothing / aceleração suave (ramp-up) para preservar engrenagens/motores.
* Usar driver H-bridge moderno para motores de maior corrente.

---

## Pinout final (tabela)

| Função        | Arduino  | L298N / HC-05 |
| ------------- | -------- | ------------- |
| Motor1 IN1    | D5       | IN1           |
| Motor1 IN2    | D6       | IN2           |
| Motor2 IN3    | D9       | IN3           |
| Motor2 IN4    | D10      | IN4           |
| HC-05 TX → RX | D2       | HC-05 TX      |
| HC-05 RX ← TX | D3*      | HC-05 RX      |
| HC-05 VCC     | 5V       | VCC           |
| GND comum     | GND      | GND           |
| Fonte motores | +V (5V) | L298N     |

* Use divisor de tensão no sinal Arduino D3 → HC-05 RX.

---

## Licença

Sinta-se à vontade para usar, adaptar e compartilhar este projeto. Se publicar, credite o autor conforme desejar.


