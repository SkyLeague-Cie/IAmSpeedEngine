// Included inside the real registry. No public seal factory or test stand-in.
    std::shared_ptr<const FNativeRequestedFrameAdmissionV2> FrozenNativeV2;
    std::uint64_t NativeV2ControlOrdinal=0;
    std::map<std::uint64_t,EBoundaryOperation> NativeV2GlobalControls;
    using FNativeV2AdapterOperation=std::function<bool(const FSessionDescriptor&,const std::shared_ptr<const FNativeRequestedBindingV2>&)>;
    using FNativeV2MappingCheck=std::function<bool(const FSessionDescriptor&,const std::shared_ptr<const FNativeRequestedBindingV2>&,FFrameNumber)>;
    bool HasNativeV2Sessions()const noexcept
    {return std::any_of(Sessions.begin(),Sessions.end(),[](const auto& P){return bool(P.second.NativeV2Binding);});}
    void MarkNativeV2Retirements(ERequestedRetirementReasonV2 Reason)noexcept
    {
        for(auto& P:Sessions)if(P.second.NativeV2Binding)
        {P.second.NativeV2Retire=Reason;P.second.Phase=ESessionPhase::CancelPending;}
    }
    void NativeV2Completed(FSession& S,FFrameNumber N)noexcept
    {
        ++S.NativeV2CompletedFrames;
        std::lock_guard<std::mutex> Lock(Commands->Gate);auto I=Commands->Records.find(S.NativeV2Command);
        if(I!=Commands->Records.end()&&I->second.Receipt.NativeV2)
        {auto& R=*I->second.Receipt.NativeV2;R.CompletedFrames=S.NativeV2CompletedFrames;R.ControlFrame=N;}
    }
    bool ServiceNativeV2Claims(const FNativeV2AdapterOperation& Claim,const FNativeV2AdapterOperation& Neutralize)
    {
        if(!IsWorker()||Pending||FrozenRequestedFrame)return false;
        for(auto& P:Sessions)
        {
            auto& S=P.second;if(!S.NativeV2Binding||S.NativeV2Claimed||Terminal||S.NativeV2Retire!=ERequestedRetirementReasonV2::None)continue;
            const auto Closed=S.NativeV2Binding->Seal()->ReadClosure();
            bool Good=!Closed.Host&&!Closed.Slot&&!Closed.Socket&&!S.NativeV2Binding->Seal()->Expired(std::chrono::steady_clock::now());
            std::shared_ptr<FInputRegistryView> Next,Failed;
            // Reserve both published projections before the game can claim.
            // Nothing that allocates may leave a successful claim unpublished.
            try
            {
                if(Good)
                {
                    Next=std::make_shared<FInputRegistryView>(*View);
                    if(Next->Version==UINT64_MAX)Good=false;
                    else
                    {
                        ++Next->Version;Next->Bindings.push_back(S.Description);
                        std::sort(Next->Bindings.begin(),Next->Bindings.end(),[](const auto& A,const auto& B){return A.Id<B.Id;});
                        Failed=std::make_shared<FInputRegistryView>(*Next);Failed->Terminal=true;
                        Good=Claim&&Claim(S.Description,S.NativeV2Binding);
                    }
                }
            }
            catch(...){Good=false;}
            if(!Good){S.NativeV2Retire=ERequestedRetirementReasonV2::ClaimFailed;continue;}
            S.NativeV2Claimed=true;S.Phase=ESessionPhase::Active;TerminalView=std::move(Failed);View=std::move(Next);
            std::lock_guard<std::mutex> Lock(Commands->Gate);auto& R=Commands->Records.at(S.NativeV2Command).Receipt;R.Status=EBoundaryStatus::Applied;Outcomes(Commands->Records.at(S.NativeV2Command));
        }
        return ServiceNativeV2Retirements(Neutralize);
    }
    bool ServiceNativeV2Retirements(const FNativeV2AdapterOperation& Neutralize)
    {
        if(!IsWorker()||Pending||FrozenRequestedFrame)return false;
        std::vector<std::uint64_t> Retire;
        for(auto& P:Sessions)if(P.second.NativeV2Binding)
        {
            auto& S=P.second;const auto& Seal=*S.NativeV2Binding->Seal();const auto C=Seal.ReadClosure();
            if(S.NativeV2Retire==ERequestedRetirementReasonV2::None)
            {
                if(C.Host)S.NativeV2Retire=ERequestedRetirementReasonV2::HostClosed;
                else if(C.Slot)S.NativeV2Retire=ERequestedRetirementReasonV2::SlotClosed;
                else if(C.Socket)S.NativeV2Retire=ERequestedRetirementReasonV2::SocketClosed;
                else if(Seal.Expired(std::chrono::steady_clock::now()))S.NativeV2Retire=ERequestedRetirementReasonV2::Expired;
            }
            if(S.NativeV2Retire!=ERequestedRetirementReasonV2::None)Retire.push_back(P.first);
        }
        for(auto Id:Retire)
        {
            auto& S=Sessions.at(Id);auto Next=std::make_shared<FInputRegistryView>(*View);if(Next->Version==UINT64_MAX)return false;
            Next->Bindings.erase(std::remove_if(Next->Bindings.begin(),Next->Bindings.end(),[&](const auto& B){return B.Id==Id;}),Next->Bindings.end());++Next->Version;
            auto Failed=std::make_shared<FInputRegistryView>(*Next);Failed->Terminal=true;
            // Native/game cancellation occurs before the source is retired,
            // outside the command mutex. It must retain body registration.
            bool Neutral=false;try{Neutral=Neutralize&&Neutralize(S.Description,S.NativeV2Binding);}catch(...){Neutral=false;}
            const auto Reason=S.NativeV2Retire;const auto Close=S.NativeV2Binding->Seal()->ReadClosure();
            const bool DataRetired=S.RequestedOwner&&S.RequestedOwner->RetireAtBoundary();
            if(DataRetired){S.RequestedOwner.reset();++Destroyed;}
            {
                std::lock_guard<std::mutex> Lock(Commands->Gate);auto& R=Commands->Records.at(S.NativeV2Command).Receipt;
                R.Status=Neutral&&DataRetired?(S.NativeV2CompletedFrames?EBoundaryStatus::Retired:EBoundaryStatus::ConstructedThenRetired):EBoundaryStatus::TerminalFailure;
                if(R.NativeV2){auto& V=*R.NativeV2;V.Retirement=Reason;V.Closures=Close;V.SourceDestroyed=DataRetired;V.CompletedFrames=S.NativeV2CompletedFrames;if(!V.CompletedFrames&&DataRetired)V.Construction=ERequestedConstructionV2::ConstructedThenRetired;}
                if(S.NativeV2DetachCommand)
                {
                    auto& Detach=Commands->Records.at(S.NativeV2DetachCommand).Receipt;
                    Detach.NativeV2=R.NativeV2;
                    Detach.Status=Neutral&&DataRetired?EBoundaryStatus::Applied:EBoundaryStatus::TerminalFailure;
                }
            }
            if(!Neutral||!DataRetired)return false;
            const auto BindCommand=S.NativeV2Command,DetachCommand=S.NativeV2DetachCommand;
            NetworkAdmissions.erase(BindCommand);Sessions.erase(Id);TerminalView=std::move(Failed);View=std::move(Next);
            std::lock_guard<std::mutex> Lock(Commands->Gate);
            Outcomes(Commands->Records.at(BindCommand));
            if(DetachCommand)Outcomes(Commands->Records.at(DetachCommand));
        }
        if(!HasNativeV2Sessions()&&!NativeV2GlobalControls.empty())
        {
            std::lock_guard<std::mutex> Lock(Commands->Gate);
            for(const auto& C:NativeV2GlobalControls)
            {
                auto& R=Commands->Records.at(C.first);
                if(R.Receipt.Status==EBoundaryStatus::Pending)R.Receipt.Status=EBoundaryStatus::Applied;
                Outcomes(R);
            }
            NativeV2GlobalControls.clear();
        }
        return true;
    }
    ERequestedFrameControlV2 CaptureNativeV2BeforeFreeze(FFrameNumber N,const FNativeV2MappingCheck& Validate)
    {
        if(!IsWorker()||Terminal||Pending||FrozenRequestedFrame||FrozenNativeV2)return ERequestedFrameControlV2::Terminal;
        if(std::none_of(Sessions.begin(),Sessions.end(),[](const auto& P){return bool(P.second.NativeV2Binding);}))return ERequestedFrameControlV2::Ready;
        std::vector<FRequestedFrameBindingV2> Captures;Captures.reserve(Sessions.size());bool Needs=false;
        if(NativeV2ControlOrdinal==UINT64_MAX)return ERequestedFrameControlV2::Terminal;
        const auto Ord=++NativeV2ControlOrdinal;const auto Now=std::chrono::steady_clock::now();
        for(auto& P:Sessions)if(auto& B=P.second.NativeV2Binding)
        {
            auto& S=P.second;const auto C=B->Seal()->ReadClosure();ERequestedRetirementReasonV2 Reason=S.NativeV2Retire;
            if(C.Host)Reason=ERequestedRetirementReasonV2::HostClosed;
            else if(C.Slot)Reason=ERequestedRetirementReasonV2::SlotClosed;
            else if(C.Socket)Reason=ERequestedRetirementReasonV2::SocketClosed;
            else if(B->Seal()->Expired(Now))Reason=ERequestedRetirementReasonV2::Expired;
            else if(!S.NetworkAdmission||!S.NetworkAdmission->Service||S.NetworkAdmission->Service->IsAcquisitionClosed()
                ||!B->OnOwner()||!B->Source()->IsOpen()||!B->Source()->MatchesSource(S.NativeV2SourceAddress)
                ||!B->Source()->MatchesService(S.NetworkAdmission->Service->ReadServiceIdentity())
                ||B->Source()->Generation()!=S.Description.Network->SourceGeneration)Reason=ERequestedRetirementReasonV2::SourceClosed;
            else
            {
                bool Mapped=false;
                try{Mapped=S.NativeV2Claimed&&S.Phase==ESessionPhase::Active&&S.RequestedOwner
                    &&S.RequestedOwner->GetPhase()==ERequestedOwnerPhase::Idle&&S.RequestedOwner->GetNextFrame()==N
                    &&Validate&&Validate(S.Description,B,N);}catch(...){Mapped=false;}
                if(!Mapped)Reason=ERequestedRetirementReasonV2::MappingChanged;
            }
            if(Reason!=ERequestedRetirementReasonV2::None){S.NativeV2Retire=Reason;Needs=true;}
            {
                std::lock_guard<std::mutex> Lock(Commands->Gate);auto& R=Commands->Records.at(S.NativeV2Command).Receipt;
                if(R.NativeV2){R.NativeV2->ControlOrdinal=Ord;R.NativeV2->ControlFrame=N;R.NativeV2->Closures=C;}
            }
            Captures.push_back({B,C});
        }
        if(Needs)return ERequestedFrameControlV2::NeedsOwnerBoundary;
        // All live control checks precede every source preflight. Preflight
        // retains data only: no owner poll ordinal/filter/install/step changes.
        // Commit of the selected input still occurs after the rollback decision.
        for(auto& P:Sessions)if(P.second.NativeV2Binding)
        {
            auto& S=P.second;
            if(!S.RequestedOwner||!S.RequestedOwner->ValidateSourceAtAdmittedCutoff(N))
            {S.NativeV2Retire=ERequestedRetirementReasonV2::InvalidSelection;S.Phase=ESessionPhase::CancelPending;Needs=true;}
        }
        if(Needs)return ERequestedFrameControlV2::NeedsOwnerBoundary;
        if(!Captures.empty())FrozenNativeV2=std::shared_ptr<const FNativeRequestedFrameAdmissionV2>(new FNativeRequestedFrameAdmissionV2(N,Ord,std::move(Captures)));
        return ERequestedFrameControlV2::Ready;
    }
    bool ValidFrozenNativeV2(FFrameNumber N,const std::shared_ptr<const FNativeRequestedFrameAdmissionV2>& Token)const noexcept
    {return Token?Token==FrozenNativeV2&&Token->OnOwner()&&Token->Frame()==N:!FrozenNativeV2;}
