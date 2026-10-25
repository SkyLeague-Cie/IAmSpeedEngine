# IAmSpeed 0.4.1 — stabilisation et intégration

Version du descripteur : 401 / 0.4.1. Simulation canonique : 300 Hz.

## Contenu

- Stabilisation des contrats caméra/entrées et de leur frontière de publication.
- Roues, admission de simulation, classification des contacts et suspension.
- Cycle de vie, enregistrement et reprise des contrôleurs ; commandes de conduite.
- Conservation des ajouts Root lors de l'intégration des correctifs déjà revus.
- Journal/witness, stockage des lignes de roues et cohérence des sorties caméra.

Le lot initial de 449 fichiers qualifiés est strictement celui du candidat
`c0f56e6112664ab7a4898186cb2675f1e8d720db`. La branche de livraison reprend
leur arbre sur le vrai main IAmSpeed `a01528654c6b1cae2fdb9712f4abbb0d69f6e4f8`
et ajoute uniquement cette note dans `Docs/Releases/`. Aucun hunk C++,
coefficient physique, baseline, seuil, tolérance ou sélection de tests n'est
modifié par le conditionnement.

La première CI de la revue (PR54, run40188110361) a compilé sous Linux puis
exécuté 231 tests : 227 réussites et quatre échecs. Les fixtures Contacts1221,
EightCars1552, Grounded1368 et CanonicalCamera1217 injectent une queue caméra
explicite avec un contrat V2 de conduite seul ; elles doivent sélectionner
explicitement cet adaptateur conservé, au lieu des nouveaux slots caméra V2
du profil stock. La correction de revue configure leurs slots à -1/-1 avant
l'initialisation native. Leurs 189 assertions, commandes, nombres de frames,
voies et seuils sont conservés. La production n'est pas modifiée ; le FAIL
original reste visible et la CI doit qualifier ce nouveau SHA.

## Validation acquise

Sur le projet actif IAmSpeedDemo : compilation Windows des quatre modules,
stock WriteMetadata et découverte effective de 6574 tests. Les 21 régressions
ciblées ont ensuite donné 21 Success, 0 erreur, 0 avertissement, 0 échec,
0 ignoré ; Editor0, contrôleur0, fin de file21 et TEST COMPLETE0.
Les logs originaux sont conservés et le lecteur canonique a réellement
terminé0 avant interprétation. Le validateur exact n'a pas été assoupli.

Qualification locale2931 : reçu SHA256
`03fedb6632222e2a31a4f506d34cda2ebdfbd294bbb229c119334a8f4f07df2b` ;
assessment `fe899c1d392841ae600cb12e23bbf9ec6023cc297057a365ff7b3f66633da25b`.

La recette utilise les options stock `-locale=en-US`, `-DisablePython`,
`-DDC=(Local)` avec DDC privée et
`-ini:Engine:[ConsoleVariables]:HomeScreen.EnableHomeScreen=0`.
La dernière option écarte le panneau d'accueil Editor et sa sonde Internet,
sans filtrer les logs. La première exécution2929 demeure refusée par le
validateur strict : 21 assertions réussies mais 41 avertissements HTTP.
Les erreurs historiques fr-FR restent également conservées.

## Installation et artefacts

L'archive source contient `IAmSpeed/IAmSpeed.uplugin`, `Source/`,
`Documentation/`, la licence et les ressources amont inchangées.
Installer ce dossier sous `Plugins/IAmSpeed` d'un projet UE5.8.2, puis
reconstruire les cibles du projet avec son propre SDK. Aucun SDK Unreal,
cache, Intermediate, donnée de travail ou copie humaine n'est distribué.

Le supplément Win64Editor contient seulement les deux DLL du plugin et son
manifest de modules testés. Il est lié au BuildId Engine exact consigné
dans `binary-provenance.json` ; ce n'est pas un BuildPlugin universel pour
toutes les installations UE5.8.2. La CI SkyLeague reconstruira depuis les
sources sur son propre tuple. Les deux DLL du host Demo ne font pas partie
du plugin et ne sont pas intégrées à SkyLeague.

IAmSpeedDemo0.1 reste un hôte indépendant à300Hz, publié séparément ; sa
branche et les copies humaines existantes ne sont pas réécrites ici.

## Limites et revue finale

Ce PASS concerne ces21 tests dans le host Demo actif, pas le nouveau parent
SkyLeague, sa FullCI, ses Golds, le matériel, l'audio, le rendu, une matrice
de fréquences ou Netcode0.5. Les dettes et FAIL antérieurs restent visibles.

Le parent SkyLeague est préparé depuis son main
`5bd79722adf04e04f7de679e2ad247c7b459a9ec` avec le seul gitlink IAmSpeed
avancé ; Core/Game et les autres sous-modules restent identiques.
La demande humaine confie au reviewer physique la revue groupée, le
nettoyage de la branche, les CI sur les SHA finaux et la fusion main.
Si le reviewer modifie ou fusionne le child avec un nouveau SHA, il doit
repointer le parent et qualifier cette dernière combinaison avant fusion.
Aucun tag de release ni publication de jeu ne précède ces gates.
