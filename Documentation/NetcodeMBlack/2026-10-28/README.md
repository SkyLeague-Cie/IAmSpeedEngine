# Netcode central → MBlack, 28 octobre 2026

Le Manager local MBlack crée **Netcode IAmSpeed** et **Netcode Sky League**. Ils travaillent sur `codex/netcode-common` dans IAmSpeedEngine et SkyLeague. Ce handoff est une livraison de sources, avec qualification native à refaire ; aucun match Client 1v1 n'est acquis.

## Récupération

Le parent SkyLeague sur `codex/netcode-common` contient cette capsule et pointe le commit enfant IAmSpeed livré. Récupérer le parent et ses gitlinks exacts avant de développer. Les remotes sont `https://github.com/SkyLeague-Cie/SkyLeague.git`, `IAmSpeedEngine`, `SLCorePhysics`, `SkyLeagueModule` et `SkyClientSDK` sous la même organisation. Ne réécrire aucune branche partagée.

L'archive IAmSpeed `codex/netcode-central-20261028-prebase` conserve les neuf commits centraux originaux ET le candidat de publication non qualifié (commit be5fd5e67b4a3953b4ef51b2e0269f0ddf65a71a). Les SHAs complets, base, correspondance du rebase et tous les gitlinks sont dans `rebase-and-pins.json` et la capsule parent. Les preuves centrales sont copiées ici : leur lecture ne dépend pas de E: sur MBlack.

## Rebase réellement effectué

Neuf commits sont rebasés directement sur IAmSpeed 0.17-base `3e6fb10d2b5e40710ca644b296c5188960a6f855`. Le seul conflit de ces neuf commits est l'ajout terminal de `CaptureReplayStateFields` dans SimulationWorld.cpp : garder cette fonction 0.17, puis ajouter les deux includes PreparedWorldRestore et PreparedAccessComposition. Huit patchs sont identiques selon range-diff ; celui-ci est adapté à ce contexte. Les quatre préimages de conflit sont conservées.

Le dixième commit d'archive (publication atomique opt-in, sept fichiers) a trois conflits majeurs : 0.17 sépare ASpeedSimulation (contrat de valeurs/presentation) et ASpeedPhysicsSimulation (worker/physique). Il est **déféré**, pas appliqué à la branche commune : aucun ancien worker n'est réintroduit. Son patch complet, contrats et provenance restent dans `pending-publication/`, ainsi que dans l'archive Git. Les cinq fichiers de l'architecture worker 0.17 restent identiques à la base. Porter la publication vers le véritable owner ASpeedPhysicsSimulation et refaire ses tests avant activation. Le candidat ancien était déjà non compilé/non testé ; il ne fournissait que le couple legacy FSimulationSnapshot/FRegistryFrame, pas fullstatev3, input courant ou branche réelle.

Le diff source porté touche 82 chemins. Une ligne vide CRLF du test ConsumedTests est normalisée en LF dans le commit de capsule ; aucune modification gameplay. Comparaison de la dérivation privée 501 fichiers avec les blobs Git : seuls les sept fichiers du candidat déféré diffèrent après normalisation CRLF/LF ; aucun octet de mojibake n'a été réparé. Les deux échecs d'outillage de comparaison (décodage UTF-8 puis absence sparse prise à tort pour PNG manquant) et leurs fermetures sont conservés.

## Deux missions

1. **Netcode IAmSpeed** : faire rejoindre le même serveur dédié de la démo par deux vrais clients. Implémenter des tests unitaires des actions atomiques réelles de la voiture. Les tests portables Netcode existants vérifient transport, input, identité, registry, replay et transactions sous shims ; ils ne couvrent pas toute la physique voiture. Inventorier le contrat actif (Throttle, Brake, Steering de base, actions déclarées par le jeu), les phases Started/Triggered/Completed, puis les actions de chaque rôle sur les surfaces applicables. La preuve attendue inclut le serveur dédié réel, deux processus clients, identité/possession, input Client→serveur→Remote et états consommés autoritaires vers propriétaire ET remotes.
2. **Netcode Sky League** : porter les commits IAmSpeed qualifiés et leurs gitlinks vers SkyLeague ; démontrer plusieurs clients sur un serveur dédié SL en private match. Garder Core/Module/SDK des pins parent tant qu'un changement explicitement scoped n'est pas nécessaire. SkyClientSDK/AWS ne sont pas une autorisation de déploiement ou de modification.

Inputs locaux immédiatement appliqués à la simulation cliente prédite, avance bornée sur serveur (valeur à établir, pas inventée). Comparer les états à la même frame historique, restaurer puis rejouer les inputs suivants. Conserver frames, monde, véritable branche, identités/incarnations, contacts, mécanique, lifecycle et registre. Publier atomiquement monde/input consommé/input courant. Tout rollback localisé inclut ses dépendances causales. Ensuite matrice rôles/actions/surfaces, puis 3v3 ; bande passante après équivalence physique et rollback. Ne fabriquer aucun Reset, epoch, baisse du high-water, branche par compteur ni tolérance gameplay.

## Vérification et étapes reproductibles

- Sur le checkout exact : `git status --short`, `git submodule status --recursive`, vérifier les SHAs et LFS ; un sparse checkout n'est pas un host natif complet. Les clones centraux de migration sont sparse et ne prouvent pas la présence des assets/configs/inputs de build.
- Compiler et exécuter les suites portables avec le toolchain **local qualifié**, paramètres explicites : `python -B Tests/Netcode/Standalone/run_standalone.py --out <nouveau-dossier> --vc <racine-MSVC-locale> --windows-kits <racine-Windows-Kits-locale> --sdk-version <version-locale>`. Le script par défaut mentionne un ancien MSVC central ; ne pas le reprendre. Ce runner contient un inventaire/timeout ordinaire et n'est pas un sandbox natif ni une admission transférable.
- Revalider les tests UE existants correspondant aux changements ; notamment ProducedInputWorkerOrderTests, ProducedWheeledInputBoundaryTests, ProducedDeviceLifecycleTests, puis nouveaux tests unitaires des actions voiture. Les shims WorldSupport ne remplacent pas UE, UHT, le link ou le gameplay.
- Identifier la vraie démo et les cibles de build existantes sur MBlack. Le host central s'appelle IAmSpeedHostProject (Game/Editor) ; aucune cible Server de démo ni compilation serveur n'a été qualifiée par Netcode. Préparer la cible dédiée manquante si nécessaire, les assets et la configuration réseau, puis build/link, startup, connexion client1/client2 et assertions du serveur et des deux clients. Ne pas présenter une ligne de commande proposée comme un serveur déjà testé.
- Répéter toutes les validations après le rebase : aucun PASS 0.4.1/2883/host501/QueryTargets n'est transféré au nouveau tuple 0.17. Le contrôleur/ressources/toolchain/readset/temps et coûts doivent être établis localement ; les identités Windows, grants et capacités de Central ne décrivent pas MBlack.

## Preuves acquises et frontières rouges

Le lot historique 2883 a des tests ordinaires sur inputs réels du code et world/common sous shims (résultats exacts dans son reçu). Ce sont des preuves historiques, pas un PASS du rebase 0.17. Les anciens bancs UDP à trois processus et états synthétiques ne prouvent pas la physique UE, toutes les actions ni le multijoueur réel.

Central a compilé les images C# UBT294/shared293 et qualifié trois modes OWN de provider (406 assertions), puis Main304 QueryTargets a chargé réellement deux TargetRules (Game/Editor), 922 fichiers SDK/266 points metadata/12 valeurs registre/une énumération Include complète de deux enfants, avec fermetures. **SelectedMsvc est absent** dans ce QueryTargets. Cela prouve startup/XML/hostSDK/rules ; aucun build natif, action graph, compilateur C++, UHT, Editor, serveur, client ou restore UE n'a été exécuté pour host501.

**NoUBA n'élimine pas UBA** dans les sources UBT centrales inspectées : ExecutorFactory conserve UBAExecutor et désactive le detouring. BuildMode précrée l'exécuteur avant SkipBuild/export ; le constructeur UBAExecutor démarre un InitTask dont les callsites incluent server/storage/session factories. Cela impose une qualification de la chaîne réellement utilisée sur MBlack, pas un transfert de contournement. Ce sont des faits source de la version centrale, pas une observation d'activation native.

Correction du diagnostic : EpicGames.UBA.Utils.RegisterCrossArchitecturePath a 63 octets d'IL managé, PInvoke=false, deux appels DirectoryReference.FindCorrectCase, puis dictionnaire/getter/event. Son caractère « helper natif direct » n'était pas prouvé. Le root DirectoryReference.FindCorrectCase (17 octets IL) délègue à DirectoryUtils.FindCorrectCase. Analyse PEReader seulement, jamais Assembly.Load/Invoke du target : 17 QA du décodeur et 64 contrôles parent par acteur, Job0 et handles fermés. Le comportement filesystem du callee, événements/cctor/lazy et backend UBA reste UNKNOWN. Source319/helper320 sont conservés non compilés/non exécutés ; la migration arrête cette exploration centrale.

Restent NON ACQUIS : fullstatev3 complet ; providers voitures/balle/pads/autres ; vraie branche ; join atomique input courant+consommé+monde ; restore natif ; rollback causal avec bindings actifs ; matrice voiture/rôles/surfaces ; native UE501 ; serveur dédié et deux clients ; private match SL ; 3v3 ; bande passante après équivalence. Les défauts/FAIL/UNKNOWN, coûts et préimages restent conservés ; aucun budget, grant ou claim n'est remis à zéro.

## Non transférable et restitution

Aucun Engine/SDK, binaire/PCH/Intermediate/DDC, image C# centrale, DLL UBA/MSVC, cache, alias, lease, identité OS ou grant n'est portable dans cette capsule. Recréer les sorties à partir des sources et du toolchain local. Les reçus DATA_ONLY ne sont ni autorisations exécutables ni preuve ABI MBlack. Aucun runtime partagé ni Engine n'a été modifié.

La restitution du slot4 central et le retrait de l'automation sont attestés séparément dans la capsule parent `RESTITUTION.json` après livraison. Aucune reprise automatique du lead central ; le Manager central peut réaffecter ce slot. Les anciens artefacts/preimages sont conservés. MBlack reprend avec son manager local ; cette migration ne crée pas ses chats depuis Central.
