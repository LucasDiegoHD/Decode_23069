## Purpose

Define o comportamento observável de cada um dos seis subsistemas do robô e o padrão de
controle contínuo que cada um mantém enquanto um OpMode está ativo.

## ADDED Requirements

### Requirement: Controle contínuo do robô

O robô SHALL expor uma rotina de atualização única que, uma vez agendada como comando contínuo,
executa em toda iteração do loop até o fim do OpMode. Essa rotina SHALL limpar o bulk cache dos
hubs e então atualizar cada subsistema em ordem fixa e determinística, aplicando o estado atual
aos atuadores e publicando telemetria. O comando contínuo SHALL NOT reservar nenhum subsistema
como recurso, de modo a rodar em paralelo com os comandos de ação.

#### Scenario: Controle contínuo sobrevive a comandos de ação

- **WHEN** um comando de ação que reserva um subsistema é agendado e depois termina
- **THEN** o comando de controle contínuo nunca deixou de executar
- **AND** a telemetria dos subsistemas continuou sendo publicada durante todo o período

#### Scenario: Ordem de atualização é determinística

- **WHEN** uma iteração do controle contínuo executa
- **THEN** o bulk cache é limpo antes de qualquer leitura de sensor daquela iteração
- **AND** os subsistemas são atualizados sempre na mesma ordem

### Requirement: Drivetrain

O subsistema de drivetrain SHALL atualizar a odometria do `Follower` a cada iteração, expor a
pose atual, indicar se o robô está parado, permitir zerar as potências, restaurar a pose a
partir do armazenamento persistente e reportar a tensão da bateria. SHALL aceitar comando de
condução em campo-cêntrico com entradas de translação e rotação.

#### Scenario: Odometria atualiza a cada loop

- **WHEN** o OpMode está em execução
- **THEN** a pose reportada pelo drivetrain reflete o movimento do robô a cada iteração

#### Scenario: Condução em teleoperação

- **WHEN** o comando de condução recebe entradas dos analógicos
- **THEN** as rodas se movem de forma equivalente ao comportamento atual (mesma suavização de
  aceleração, trava de rumo e escala por tensão)

### Requirement: Intake

O subsistema de intake SHALL ter modos ligado, desligado e reverso, um modo de velocidade
reduzida e controle do motor de gatilho. O comando contínuo SHALL aplicar a potência
correspondente ao modo atual.

#### Scenario: Modo reverso curto

- **WHEN** a ação de reverso curto é acionada
- **THEN** o intake reverte por um intervalo definido e depois volta ao modo ligado

### Requirement: Vision

O subsistema de visão SHALL fornecer estimativas de pose por AprilTag (com e sem fusão de
heading), o desvio horizontal para o alvo, se há alvo visível e a distância direta ao alvo.

#### Scenario: Sem alvo

- **WHEN** a Limelight não vê nenhuma tag
- **THEN** a consulta de "tem alvo" retorna falso
- **AND** as estimativas de pose retornam vazio

### Requirement: Shooter

O subsistema de atirador SHALL manter velocidade dos volantes por controle de malha fechada
com compensação de tensão, posicionar os servos de capô por polinômio de distância, aceitar um
ajuste fino de RPM pelo operador, indicar quando está na velocidade-alvo e permitir parar.

#### Scenario: Ajuste fino de RPM

- **WHEN** o operador incrementa o offset de RPM
- **THEN** a velocidade-alvo efetiva sobe pelo mesmo passo
- **AND** o valor persiste até ser zerado

#### Scenario: Pronto para atirar

- **WHEN** a velocidade medida está dentro da tolerância da velocidade-alvo
- **THEN** a consulta de "pronto" retorna verdadeiro

### Requirement: Indexer

O subsistema indexador SHALL contar as peças em posse usando três sensores, expor a contagem
atual, se está cheio e um estado de "atirando".

#### Scenario: Contagem limitada à capacidade

- **WHEN** o número de peças detectadas atinge a capacidade máxima
- **THEN** a contagem reportada não ultrapassa a capacidade

### Requirement: LED

O subsistema de LED SHALL exibir uma cor derivada da contagem de peças do indexador, em toda
iteração.

#### Scenario: Cor segue a contagem

- **WHEN** a contagem de peças muda
- **THEN** a cor do LED muda para a cor correspondente à nova contagem
