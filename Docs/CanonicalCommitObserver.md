# Canonical commit observer for Experimental

Cette extension répond à [PR125/6388843147](https://github.com/SkyLeague-Cie/SkyLeague/pull/125#issuecomment-6388843147).
## Branche et frontière de livraison

Branche isolée : codex/replay-canonical-observer-20261024.
Base parent e2dbaea3834fc741afe69007aa7df111796206db,
Module 0bd3f36588eb8ca2483400d89b4848c1114e7cea,
IAS publié 5f2c0470c1b52f167277e06d0674322ee42fddc4,
Core 966d9c1db26daa63a43bd6e080b27e438f3da919.
Cette branche ajoute le backend Source19 requis puis Observer23 dans IAS ;
le parent conserve Module et Core exacts. Elle exclut les correctifs ModuleSource24
et les packages joueur. Ce handoff est une API source Experimental en draft.

Pour la fondation MBlack actuelle, le profil privé doit fixer
EnableReplayRecording=False dans GGameIni AVANT StartupModule/Editor.
Ce guard source ne remplace pas le reçu runtime Register GameMode recording=0.
Ne pas remplacer la vraie clé par CaptureFreeplayClips. La fondation
600 neutres puis au plus2400 de premier contact garde son cap physique ;
l'observer ne relève jamais FastMaxFramesPerRun. Une configuration6000 ne
rend pas6000 frames admissibles si le vrai driver en autorise2400.
Complete n'est jamais Saved20s : Epoch/EndFrame et la notice Saved corrélée
du writer restent nécessaires. Aucun faux ReplayBody, sink ou archive n'est ajouté.

Pour compiler la fixture depuis la racine parent (sortie neuve explicite) :
cl.exe /nologo /std:c++20 /EHsc /W4 /WX /MD /Od /I Plugins/IAmSpeed/Source/IAmSpeed/World/Simulation /Fo<sortie-neuve>/observer.obj /Fe<sortie-neuve>/observer.exe Scripts/Tests/CanonicalCommitObserverCoreChecks.cpp
Ce test n'a besoin d'aucun SDK Unreal. Pour intégrer le backend, reconstruire
IAS et le tuple compatible via le workflow du projet ; UHT/Editor restent à acquérir.
Le token/config est inscrit GT dans un run encore pausé ; conserver cette pause
pendant l'inscription de tous les owners avant de libérer le premier Ready.

## Fichiers et build

Quatre nouveaux fichiers IAS :

- `CanonicalCommitObserverCore.h` : algorithme de registre et de contrôle fixe,
  C++ standard uniquement, instancié aussi par le backend réel.
- `CanonicalCommitObserver.h` : API publique et vues const des vrais commits.
- `CanonicalCommitObserverInternal.h` : hooks privés du driver.
- `CanonicalCommitObserver.cpp` : possession opaque64hosts, admission GT avec
  ACK natif, identité de worker réelle, retrait après preuve join/input owners.

Deux fichiers IAS existants ont une proposition modifiée :
`SpeedPhysicsSimulation.cpp` et `.h`. Le header ne reçoit qu'un forward/friend
et une correction de commentaire sur les échecs postcommit ; aucun champ,
UFUNCTION, UPROPERTY, vtable ou layout UCLASS n'est ajouté.

Le cpp nouveau sera compilé normalement par le module IAS. Pour le cœur portable,
un seul TU est nécessaire, sans UE/Core :

```text
cl.exe /nologo /std:c++20 /EHsc /W4 /WX /MD /Od /I Plugins/IAmSpeed/Source/IAmSpeed/World/Simulation /Fo<sortie-neuve>/observer.obj /Fe<sortie-neuve>/observer.exe Scripts/Tests/CanonicalCommitObserverCoreChecks.cpp
```

Le reçu borné ObserverCoreChecks23 atteste 163 assertions sur ce cœur exact.
Ce test instancie le cœur sur des valeurs synthétiques, sans remplacer son algorithme.
Il ne qualifie ni ACK UE, ni worker/join natif, Editor/Teaser ou prise physique20s.

## Intégration utilisateur

Inclure `IAmSpeed/World/Simulation/CanonicalCommitObserver.h`. Implémenter un
`Speed::CanonicalObservers::IObserver` C++ détenu par `std::shared_ptr`, avec :

- `Readiness() const` : Waiting/Ready/Complete/Failed, sur le worker propriétaire.
- `OnCommitted(Receipt, Values)` : Continue/Complete/Failed après le commit réel.

Préparer le stockage borné et toute donnée native de configuration avant
l'inscription. Le callback ne fait ni IO, attente, join, lookup UObject, setter
ou injection d'input. Les vues Payload/PresentationBodies/PreparedPresentationOutputs
empruntent les valeurs préparées locales acceptées par cette publication et
expirent au retour. Elles ne pointent pas sur le slot publié : le buffer estampille
seulement ses copies, donc les sorties préparées gardent leur PublicationSerial
local (normalement zéro). Le vrai serial publié, les metadata et les hashes
existants sont dans le reçu ; utiliser exclusivement ce reçu comme témoin de serial. Le callback peut
mettre à jour seulement son propre stockage préalloué. Il ne reçoit aucun
UObject, adapter de corps ou interface mutable de simulation.

SurGT, avant frame0/première publication et avant le premier contrôle Ready,
appeler `Register(Driver, Config, Observer, Handle, Failure)` avec :

- `Config.Generation = Driver.GetInputWorkerGeneration()` réellement préparée.
- `Config.OwnerToken` non nul, propre au propriétaire et unique dans le registre.
- `Config.MaximumCommittedFrames` explicite1..6000, compté depuis frame0.

Une politique600 frames neutres puis2400 de contact peut donc choisir une borne
totale3000 si c'est son sens existant ; l'extension ne réinterprète pas ce cap.
Le futur run6000 choisit sa propre borne. Aucun DefaultGame.ini ou Saved n'est
modifié par cette API. Une inscription refusée n'est ni Ready ni un run qualifié.

`Register` acquiert une vraie suspension du service de boundary, ou prouve
l'absence de worker avant son démarrage. Il exige aucune publication passée et
aucun rollback déjà admis. Les8slots sont ensuite scellés au premier contrôle
du worker ; remplacement, ajout tardif, owner dupliqué, mauvaise génération ou
capacité dépassée sont refusés. Le premier appel est attribué à l'identité de
thread capturée dans la véritable lambda du worker, pas à un thread GT arbitraire.

## Décision et ordre physique

Ready/Waiting/Complete/Failed sont combinés explicitement avec le contrôle des
participants existants. Failed est prioritaire ; Complete arrête avant le frame
suivant ; Waiting bloque l'admission. Un observer Ready peut être le contrôleur
d'un run sans s'inscrire dans Bodies. Sans registre, la décision existante reste
inchangée.

Le hook postcommit vient exactement après : publication/CommitCanonicalFrame
des participants, CompleteAll des inputs, notifications des participants et
`ServiceReplayCapture(Snapshot)`. L'appel du recorder n'est pas une preuve de
Saved : le writer et ses notices corrélées restent distincts. Les slots reçoivent
tous ces valeurs acceptées du même commit et son reçu, même si un précédent
callback complète ou échoue. Aucun stamping local ni changement du slot/hash/sink
n'est ajouté. Le scellement Configuring vers Running utilise aussi un CAS pour
ne pas réactiver un registre Failed concurrent. Tous les Result::Failed réels du
driver sont propagés au registre, y compris avant RunCanonicalFrames et sans
relever le cap physique existant ; le premier motif d'échec est conservé.

La borne physique et Complete sont testés aussi dans les lots de
`RunCanonicalFrames(N)`. Refus/exception est terminal. Les transitions Running vers Complete utilisent un CAS ;
un Failed concurrent ou une génération Retired ne peuvent pas être écrasés. Le reçu garde le fait que
le frame est déjà publié, et le compteur du driver le compte une fois avant de
sortir ; aucun Restore/rollback ne l'annule ou ne le rejoue. Le registre est
exclu des hashes, payloads, archives et listes Bodies ; aucun sink ou producteur
de présentation n'est remplacé.

## Lifetime et retrait

Le registre possède le shared storage jusqu'au véritable stop/join du worker
et à `InputOwnersRetiredAfterJoin()` pour la même génération. Le GT appelle
d'abord l'API native existante `JoinOwnedSimulationForInputTeardown()` et vérifie
son succès, puis `RetireAfterJoin(Driver, Handle, Failure)`. Le token est invalidé
avant destruction du dernier storage retenu. Le hook EndPlay effectue également
un retrait global uniquement après la même preuve réelle.

Un retrait avant join, depuis un callback, de mauvaise génération ou avec un
handle superseded est refusé. Changer de mode/génération ou admettre un rollback
sous ce registre est refusé ; retirer l'ancien propriétaire d'abord. Aucun
booléen de preuve venant d'une archive ou de l'utilisateur n'entre dans l'API UE.
Les booléens de preuve du cœur portable sont internes et alimentés par ces
témoins natifs ; leurs fixtures ne sont pas une attestation de join OS.

## Fixtures et frontières non acquises

`CanonicalCommitObserverCoreChecks.cpp` exerce le vrai template de production :
frame0, Ready/Waiting, Complete de contact ou de borne dans un lot, priorité de
Failed/exception, conservation du reçu déjàcommité, preuves participants/inputs/
recorder, ancien token/génération, capacité, retrait avant/après join et réentrance.
Il n'invente pas un algorithme de registre ni un événement matériel. Les 163 assertions portables ont effectivement réussi ; aucune réussite du
backend UE, du worker/join, d’Editor/Teaser ou d’une prise20s n’en est déduite.

Le raccord des reçus de contact propres au host MBlack, les interdictions du
callback C++, le séquençage GT/worker/teardown et UHT/Editor exigent encore leurs
tests d'intégration. Complete n'est pas une prise20s Saved. Si la demande GT de
sauvegarde arrive après la sortie du worker, elle doit emprunter un vrai drain
retired admis pour cette génération ; cette extension n'ajoute ni frame fictif
ni faux service pour terminer l'écriture.
