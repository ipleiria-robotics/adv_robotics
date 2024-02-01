# Instalação e configuração da distribuição Linux

_**NOTA: Fora da aula pode utilizar o Teams ou o e-mail para colocar dúvidas e obter respostas mais rapidamente sobre este assunto. Caso tenha alguma dificuldade, sugere-se que contacte o docente no horário da atendimento, pessoalmente ou através dos canais acima, para não perder demasiado tempo com a instalação/configuração do sistema.**_

O desenvolvimento prático na UC de Robótica Avançada é baseado no ROS2. Não obstante hoje em dia este já ser suportado em Windows, tipicamente este tipo de sistemas é implementado em ambiente Linux, sendo também essa a abordagem utilizada em Robótica Avançada. Em 2023/2024, em Robótica Avançada, a distribuição de Linux instalada nos PCs do Laboratório de Simulação de Sistemas é o Kubuntu 22.04.3 (Jammy Jellyfish). Não sendo obrigatório utilizar o Kubuntu, sugere-se fortemente que, por questões de compatibilidade, utilize uma versão do Ubuntu 22.04, que poderá ser o Kubuntu, o Ubuntu, o Xubuntu, etc., sendo o Xubuntu a que consome menos recursos. No final deste documento encontra as ligações para todo o software referido neste documento, incluindo o Kubuntu, como o Xubuntu.

Existem três formas aconselhadas para instalar a distribuição, resumidas na tabela seguinte.

|Tipo de Instalação | Ambiente | Vantagens | Desvantagens |
|:-----------------:|:--------:|:---------:|:------------:|
|[Windows Subsystem for Linux 2 (WSL2)](#1-windows-subsystem-for-linux-2-wsl2) | Windows 10/11 | Facilidade de utilização e integração entre o Windows e o Linux. | Alguma perda de performance, embora reduzida em geral e mais elevada a nível da visualização 3D. Só disponível para versões do Windows >= 10 2004 (Build 19041).|
|[Máquina Virtual](#2-máquina-virtual) | Windows (7/8/10/11) | Facilidade de instalação e remoção. | Maior perda de performance, quer em a nível de visualização 3D, quer em geral. Mais adequada para computadores com pelo menos 4 cores e 8Gb de memória.|
|[Máquina Real](#3-máquina-real) | Linux Ubuntu 22.04.3 | Máxima performance. | Maior complexidade de instalação e maior dificuldade em gerir a ligação entre o Windows e o Linux, não podendo correr os dois sistemas em simultâneo.|

Cada uma destas hipóteses está mais detalhada nas secções que se seguem.

# 1. Windows Subsystem for Linux 2 (WSL2)

Esta é a opção recomendada se o seu PC tem poucos recursos e não tem interesse em instalar o Linux a par do Windows. A Microsoft tem vindo a desenvolver um sistema baseado em Linux que permite correr as aplicações de Linux mais facilmente num ambiente Windows. Esta abordagem utiliza menos recursos que uma máquina virtual tradicional e permite trabalhar num ambiente mais próximo do Windows. No caso particular desta UC iremos utilizar o WSL2.

## 1.1 Instalação do WSL2

Para poder usar o WSL2 tem que ter a versão do Windows 10 2004 (Build 19041) ou superior, como pode confirmar [aqui](https://learn.microsoft.com/en-us/windows/wsl/install). Proceda da seguinte forma para instalar o WSL e o Ubuntu:
1. Clique no menu iniciar e abra a opção "Ativar ou desativar funcionalidades do Windows" (pode pesquisar por "Funcionalidades" para encontrar esta opção);
2. Na lista de funcionalidades disponíveis, ative a opção "Subsistema Windows para Linux" e a opção "Virtual Machine Platform". Ser-lhe-á solicitado que reinicie o PC para concluir a instalação;
3. Após reiniciar o PC, abra a loja do Windows (Microsoft Store), aceda ao [Ubuntu 22.04.3](https://www.microsoft.com/store/productId/9PN20MSR04DW) e obtenha/instale a aplicação (esta é a distribuição de Linux que será utilizada);
4. Após concluir a instalação do Ubuntu, abra a aplicação através do menu iniciar do Windows. Nessa altura ser-lhe-á solicitado que introduza um nome de utilizador e palavra-passe, para a configuração do Ubuntu (não se esqueça dessa combinação de nome de utilizador e palavra-passe);
5. Após confirmar que a instalação foi concluída com sucesso, feche a janela do Ubuntu e abra a PowerShell como administrador, clicando com o botão direito do rato no menu iniciar, seguido da opção "Windows PowerShell (admin)" (caso tenha instalado o Terminal do Windows, pode escolher a opção "Terminal (Admin)");
6. Na janela da Powershell, corra o comando "wsl -l -v" para confirmar a versão do WSL utilizada;
7. Caso a versão indicada no passo anterior seja a "1", corra o comando `wsl --set-version Ubuntu-22.04 2` para mudar para a versão 2. Se a versão já for a 2, ou se a mudança para a versão 2 (verifique com o comando do ponto 6) tiver sido concluída, pode saltar os passos 8, 9 e 10. Se a a mudança para a versão 2 não tiver sido concluída, continue para o passo 8;
8. Caso no final do passo 7 o WSL2 não tenha ficado ativo, tal poderá ser devido à necessidade de uma configuração adicional para usar o WSL2. Nesse caso, descarregue e instale a atualização disponível [aqui](https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi);
9. Reinicie o PC e confirme que tem a opção de virtualização ativada na BIOS. Caso o seu PC não suporte virtualização, não poderá utilizar o WSL2, pelo que nesse caso deve optar pela opção da [máquina virtual](#2-máquina-virtual) ou da [máquina real](#3-máquina-real);
10. Confirme novamente a versão do WSL utilizada com o Ubuntu. Se necessário, corra novamente o passo 7.
11. De modo a garantir que está a utilizar a versão mais atual do WSL2, corra o seguinte comando na consola do Windows (aberta no passo 5 com acesso de administrador): `wsl.exe --update`.

Note que pode aceder à pasta "HOME" do Linux usando o Windows Explorer com o caminho `\\wsl.localhost\Ubuntu-22.04\home\USER`, onde `USER` deve ser substituído pelo nome de utilizador que especificou na instalação do Linux.

Proceda agora com a instalação de outras aplicações no Windows que irão ser úteis em Robótica Avançada, seguindo a próxima secção.

## 1.2 Aplicações adicionais a instalar no Windows

Aconselha-se a instalar o [Notepad++](https://notepad-plus-plus.org/), caso pretenda editar no Windows ficheiros guardados no Linux.

Tem ainda que instalar o [Microsoft Visual Studio Code](https://code.visualstudio.com/) no Windows e, dentro do Visual Studio Code, instalar a extensão [WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl) publicada pela Microsoft.

Concluída a instalação da distribuição de Linux, interessa agora instalar as aplicações específicas para Robótica Avançada. Para tal, siga os passos indicados na [Secção 4](#4-instalação-e-configuração-do-software-adicional).

# 2. Máquina Virtual

Caso seja utilizador de Windows com um PC suficientemente recente (< 5 anos), esta é a uma boa opção, pois permite-lhe uma instalação mais segura e utilização prática. Naturalmente que tal acontece à custa de uma diminuição de performance, o que poderá implicar alguma lentidão em trabalhos mais pesados. Se tiver um PC com processador superior ou igual ao i5 e placa gráfica da AMD ou NVIDIA, ou uma Intel recente, a performance deverá ser aceitável, mesmo em trabalhos com simulaçõo 3D.

A máquina virtual sugerida é o [VMware Player](#5-ligações-relevantes), a qual é gratuita para uso não comercial. Alternativamente poderia ser utilizado o VirtualBox, no entanto, na última versão que foi estudada, este apresentava um pior desempenho em aplicações 3D, podendo limitar a realização de alguns trabalhos. A utilização do VMware Player também não é isenta de problemas, dado que virtualizar a simulação 3D consome muitos recursos computacionais.

Uma máquina virtual é uma aplicação que permite correr um sistema operativo num ambiente virtual, podendo assim correr outros sistemas com Windows, Linux ou Mac dentro de qualquer um desses sistemas. Neste ambientes virtuais o Host (anfitrião) é o sistema operativo que alberga a máquina virtual, enquanto o Guest (convidado) é o sistema operativo que corre na máquina virtual. O WSL2 também implementa uma máquina virtual, mas considerada _lightweight_.

Dado que a máquina virtual consume muitos recursos, aconselha-se neste caso a utilização do Xubuntu para um melhor desempenho, mas poderá utilizar uma das outras versões indicadas.

Relativamente à instalação da máquina virtual, tem duas opções:
1. Pode descarregar uma máquina previamente preparada pelo docente (processo detalhado na [Secção 2.1](#21-instalação-fornecida-pelo-docente)) ;
2. Instalar a máquina virtual de raiz (processo detalhado na [Secção 2.2](#22-instalação-da-máquina-virtual-de-raiz)). A via mais rápida e simples é a primeira, mas caso queira passar pelos passos de instalação do Linux, pode seguir a segunda via (instalação de raiz). Em ambos os casos, é possível depois copiar ficheiros entre a máquina virtual e o sistema anfitrião (usando Copy&Paste), bem como partilhar pastas entre os dois sistemas, como explicado na [Secção 4](#4-instalação-e-configuração-do-software-adicional). 

## 2.1 Instalação fornecida pelo docente
Para utilizar a instalação previamente preparada pelo docente, siga os passos abaixo:

1. Descarregue o [Vmware Player](#5-ligações-relevantes) clicando no botão "Download for free", seguido da opção "Go to downloads" e, depois, da opção "Download now" para o seu sistema operativo;
2. Instale o Vmware Player a partir do ficheiro descarregado (aceite as opções por omissão);
3. Descarregue a máquina [virtual fornecida pelo docente](TODO);
4. Descomprima o ficheiro descarregado (máquina virtual) para a pasta onde estão guardadas as máquinas virtuais (no caso do docente, a pasta `C:\Users\hugoc\Documentos\Virtual Machines\`), ou uma outra pasta qualquer à sua escolha. Após descompactar o ficheiro, irá ocupar cerca de 25Gb de armazenamendo no disco;
5. Abra o Vmware player. Caso apareça informação sobre o VMware Workstation escolha "Skip this version";
6. Clique em Player --> File --> Open... e abra, na pasta onde guardou a máquina virtual, o ficheiro "Xubuntu 22.04 64-bit AR\Xubuntu 22.04 64-bit AR.vmx";
7. Na mensagem que surge a questionar se a máquina virtual foi movida ou copiada, escolha a opção "I moved it";

O nome de utilizador é `robotics` e a palavra-passe é `robotics`.

Neste caso a máquina virtual está totalmente configurada, pelo que não necessita de fazer mais nada, exceto se quiser ter pastas partilhadas entre o Windows (_host_) e o Linux (_guest_) (note que é possível fazer _Copy&Paste_ de conteúdos e ficheiros entre o _host_ e o _guest_, independentemente de partilhar ou não as pastas). Para ativar as pastas partilhadas entre o Windows e o Linux clique, dentro da janela do VMware, no menu _Player_ --> _Manage_ --> _Virtual Machine Settings_ e  seleccione o separador "Options", seguido da opção "Shared Folders". Ative a opção "Always Enabled" e clique em "Add" para adicionar a pasta a partilhar. As pastas criadas ficarão imediatamente disponíveis na máquina virtual em `/mnt/hgfs`.
Neste momento está apto a seguir os tutoriais das aulas laboratoriais, não precisando de fazer mais nada em termos de instalação/configuração.

## 2.2 Instalação da máquina virtual de raiz

Para proceder à instalação do sistema na máquina virtual, siga os seguintes passos:

1. Descarregue o [Vmware Player](#5-ligações-relevantes) clicando no botão "Download for free", seguido da opção "Go to downloads" e, depois, da opção "Download now" para o seu sistema operativo;
2. Descarregue a versão [22.04.3 do Xubuntu 64 bit](#5-ligações-relevantes);
3. Instale o Vmware Player a partir do ficheiro descarregado (aceite as opções por omissão);
4. Abra o Vmware player. Caso apareça informação sobre o VMware Workstation escolha "Skip this version";
5. Clique em "Create a New virtual Machine";
6. Selecione a opção "Installer disc image file (iso)" e escolha o ficheiro ISO do Xubuntu que descarregou acima. Clique em Next;
7. Escolha um nome de utilizador e palavra-passe (não a pode esquecer) e clique em Next;
8. Escolha um nome para a máquina virtual e clique em Next;
9. Altere o tamanho do disco para 25Gb ou superior e selecione a opção "Store virtual disk as a single file". Clique em Next;
10. Na configuração do hardware a máquina virtual, acessível clicando em "Customize Hardware...", deverá ter 3GB de memória ou mais (idealmente 4096 Mb caso tenha 8 Gb ou mais de memória) e 1 processador ou mais (idealmente 2 se tiver 4 ou mais _cores_). Clique em "Finish" para iniciar a instalação, processo este que poderá 10 a 20  minutos dependendo do seu hardware;
11. Caso surja uma janela com o título "Removable devices", poderá fechar a mesma sem problemas;

Terminada a instalação, é necessário proceder à configuração do sistema:

1. Se após a instalação o teclado não estiver correto, pode reconfigurar o mesmo. Para tal aceda ao menu iniciar, pesquise por "Settings Manager" e clique no mesmo. Clique em "Keyboard". No separador Layout remova a opção "Use system defaults", clique em "Edit" e selecione a opção "Portuguese (no dead keys)", seguido de Close. Poderá levar alguns segundos até poder prosseguir;
2. Ainda na janela "Settings Manager", pesquise por "Time and Date", clique no ícone que aparece, depois clique em "Unlock", introduza a palavra-passe que configurou anteriormente,  altere a "Time zona" para Europe/Lisbon e clique "Close" para terminar;
3. É possível fazer Copy&Paste de conteúdos e ficheiros entre o _host_ e o _guest_, mas  poderá também ser útil partilhar uma pasta entre o Windows e o Linux. Para tal, clique no menu "Player" (caso não esteja visível, o menu aparece quando desloca o rato para o topo da parte central da janela da maquina virtual) --> "Manage" --> "Virtual Machine Settings". Selecione o separador "Options", seguido da opção "Shared Folders". Ative a opção "Always Enabled" e clique em "Add" para adicionar a pasta a partilhar. O campo "Host path" diz respeito à pasta do Windows que quer partilhar com a máquina virtual Linux. O campo "Name" corresponde ao nome que a pasta terá no Linux (máquina virtual). Após preencher os campos clique "Next" seguido de "Finish". A pasta criada ficará imediatamente disponível no Linux em `/mnt/hgfs`;
4. Ainda na janela do ponto anterior, selecione a opção "VMWare Tools", ative a opção "Synchronize guest time with host";
5. Por omissão o Xubuntu bloqueia o ecrã ao fim de 5 minutos. Caso queira alterar esse tempo ou desabilitar esse bloqueio, aceda no menu iniciar a "Settings", seguido de "Screensaver Preferences", e altere aí as configurações que achar necessárias. 

Terminada a instalação da distribuição, é agora necessário instalar o software de base utilizado nas aulas, de acordo com a [Secção 4](#4-instalação-e-configuração-do-software-adicional).


# 3. Máquina Real

A instalação em máquina real, "ao lado do Windows", é a opção que permite ter melhor performance, fazendo utilização plena dos recursos do PC. No entanto esta implica alguns cuidados durante a instalação, sobretudo por utilizadores menos experientes, obrigando ainda a uma comutação entre sistemas para utilizadores do Windows, a qual poderá não ser prática. Apenas recomendo esta opção se se sentir à vontade com estas operações ou tiver um PC com mais de 6 anos.

Para instalar a distribuição deverá primeiro garantir que tem um CD ou uma PEN USB com a distribuição gravada (pode usar a aplicação [balenaEtcher](#5-ligações-relevantes)  para gravar a PEN USB a partir da ISO do Linux). Deverá ainda garantir que tem pelo menos 2Gb de memória e 25Gb de disco para o Linux (idealmente já em espaço livre, i.e., já particionado). Arranque desse CD/PEN e siga os seguintes passos (baseados no Kubuntu 22.04, mas serão semelhantes para outras versões do Ubuntu):

1. No menu inicial escolha a opção de instalação;
2. Selecione "Download updates while installing" e "install third-party software" após o arranque;
3. Quando especificar o nome de utilizador e palavra-passe, ative a opção "Log in automatically". Não se esqueça da palavra-passe que especificar e use apenas letras e números no nome do utilizador (não use acentos nem espaços);
4. Este é um passo crítico, pelo que dev escolher muito cuidadosamente o particionamento do disco. Contacte o docente em caso de dúvida (um erro aqui pode levar à perda de todos os dados do PC);
5. Após a instalação, realize os seguintes passos:
    * Abra "System Settings" no menu iniciar, clique em "Locale" e altere o país para Portugal. Faça _logout_ e _login_ para as alterações fazerem efeito;
    * Abra o menu iniciar a opção "Driver Manager" (alternativamente poderá pressionar Alt+F2 e escrever "Driver Manager"). Se houver software disponível, ative-o (poderá ter que reiniciar o PC para as alterações fazerem efeito).

Terminada a instalação da distribuição, é agora necessário instalar o software de base utilizado nas aulas, de acordo com a [Secção 4](#4-instalação-e-configuração-do-software-adicional).

# 4. Instalação e configuração do software adicional

Para facilitar a instalação do software de base necessário para Robótica Avançada, o docente preparou um conjunto de _scripts_. Assim, para instalar o software basta que siga os seguintes passos (note que estes passos incluem a descarga e instalação e de cerca de 2Gb de software):

1. Abra um terminal no Linux ou, caso esteja a usar o WSL, abra o Ubuntu.
2. Execute, no terminal, o 1º comando que se segue, caso **não** esteja a usar o WSL, ou o 2º comando que se segue, caso esteja a usar o WSL: 
```bash
wget https://github.com/ipleiria-robotics/adv_robotics/raw/master/scripts/install_AR.sh
```
```bash
wget https://github.com/ipleiria-robotics/adv_robotics/raw/master/scripts/install_AR_WSL.sh 
```
3. Ainda no mesmo terminal, execute agora o comando `bash install_AR.sh` (ou o comando `bash install_AR_WSL.sh`, caso esteja a usar o WSL);
4. Siga as instruções exibidas no terminal, inserindo a palavra-passe, sempre que solicitada, e respondo sim (Y) ou com o ENTER às questões que surjam. Note que, dependendo da velocidade de internet, este passo pode demorar significativamente;

Terminada esta instalação verifique no terminal que não ocorreu nenhum erro durante a execução dos scripts. Caso tenha surgido algum erro, além do indicado abaixo, volte a correr o script (passo 3). Caso o(s) erro(s) não sejam resolvidos, contacte o docente. O erro que se segue pode ser ignorado sem problemas.
```bash
WARNING:colcon.colcon_ros.task.ament_python.build:Package 'py_trees_ros_viewer' doesn't explicitly install a marker in the package index (colcon-ros currently does it implicitly but that fallback will be removed in the future)
--- stderr: py_trees_ros_viewer
/home/user/ros/build/py_trees_ros_viewer/setup.py:5: DeprecationWarning: The distutils package is deprecated and slated for removal in Python 3.12. Use setuptools or check PEP 632 for potential alternatives
  from distutils import log
```
De seguida deve reiniciar o PC, ficando este pronto para a realização dos trabalhos práticos.

No caso de estar a utilizar uma máquina virtual e queira partilhar pastas entre o sistema operativo anfitrião mas ainda não o tenha feito, siga as indicaçõe no final da [Secção 2.1](#21-instalação-fornecida-pelo-docente).

# 5. Ligações relevantes

* [Kubuntu 22.04.2](https://cdimage.ubuntu.com/kubuntu/releases/22.04.3/release/kubuntu-22.04.3-desktop-amd64.iso)
* [Xubuntu 22.04.3](https://cdimage.ubuntu.com/xubuntu/releases/22.04.3/release/xubuntu-22.04.3-desktop-amd64.iso)
* [Vmware Player](https://www.vmware.com/products/workstation-player.html)
* [balenaEtcher](https://etcher.balena.io/)