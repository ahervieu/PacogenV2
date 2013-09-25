:-use_module(library(lists)).
:-use_module(library(samsort)).
:-use_module(library(atts)).
:-use_module(library(clpfd)).
:- attribute ftAttr/1 , lstAttr/2.

verify_attributes(_, _, _).

/*
Final file
result of the concatenation of globalconstraint, tools, and pairwise generator
*/
/*
Term to read the model.
*/
printlst([]).
printlst([A|R]):-writeln(A),
                printlst(R).


introspection(Labelling,Minimization,AllDif,Sort,SymetricBreaking):-
        write('Labelling :'),
        write(Labelling),
        write(' Minimization :'),
        write(Minimization),
        write(' AllDif :'),
        write(AllDif),
        write(' Sort :'),
        write(Sort),
        write(' SymetricBreaking :'),
        write(SymetricBreaking),
        write('\n').





save:-save_program('/home/aymeric/Bureau/XP2013_Journal_Rev/Tableau_CMp_NewHeurisitic/Pacogen.sav').
run2:-
        openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName),(        
        solverGen(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName,Time,Size,Pairinfo),!,
        openfile(PairInformations2,Attributes2,Features2,CtrList2,MatrixSize2,ModelName2,Labelling2,Minimization2,AllDif2,Sort2,SymetricBreaking2,FileName2),!,
        solverGen2(PairInformations2,Attributes2,Features2,CtrList2,Size,ModelName2,Labelling2,Time,AllDif2,Sort2,SymetricBreaking2,FileName2,Pairinfo);        writeStat0(FileName,'./stats2.txt'), 
        open('./stats.txt', append, Stream),nl(Stream),close(Stream),
        open('./stats2.txt', append, Stream),nl(Stream),close(Stream)).

/*
user:runtime_entry(start)  :-
             openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName),
        (        
         solverGen(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName,Time,Size,Pairinfo),!,
           openfile(PairInformations2,Attributes2,Features2,CtrList2,MatrixSize2,ModelName2,Labelling2,Minimization2,AllDif2,Sort2,SymetricBreaking2,FileName2),!,
         solverGen2(PairInformations2,Attributes2,Features2,CtrList2,Size,ModelName2,Labelling2,Time,AllDif2,Sort2,SymetricBreaking2,FileName2,Pairinfo);        writeStat0(FileName,'./stats2.txt'), 
       open('./stats.txt', append, Stream),nl(Stream),close(Stream),
       open('./stats2.txt', append, Stream),nl(Stream),close(Stream)).

*/
user:runtime_entry(start)  :-
             openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName),    
         solverGen(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName,Time,Size,Pairinfo),!,
               open('./stats.txt', append, Stream),nl(Stream),close(Stream).


run22 :-        openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName),
        (solverGen(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,_,AllDif,Sort,SymetricBreaking,FileName,Time,Size,Pairinfo);
       open('./stats.txt', append, Stream),nl(Stream),close(Stream)).

test:-        openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName),
       solverGen(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,_,AllDif,Sort,SymetricBreaking,FileName,Time,Size,Pairinfo).
openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName) :-
        open('model.txt', read, Stream),
        read(Stream,[PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName]),close(Stream).


test2:-openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName),
       borneRec(Features),
       callRec(CtrList),
       printlst(CtrList),
       writeln(Features).
/*
invalidPairDetectionMechanism(Pairwise Constraint List, Threshold, InvalidPairs)

*/
invalidPairDetectionMechanism([],_,[]).
invalidPairDetectionMechanism([pairwise(I,(L1,L2),(V1,V2))|R],Threshold,[pairwise(I,(L1,L2),(V1,V2))|R2]):-
        I >= Threshold,!,
        invalidPairDetectionMechanism(R,Threshold,R2).

invalidPairDetectionMechanism([_|R],Threshold,R2):-
        invalidPairDetectionMechanism(R,Threshold,R2).


invalidPairMechanismDetection([], _,[]).
invalidPairMechanismDetection([pairwise(I,(L1,L2),(V1,V2))|R2], M,[b((Rank1,Rank2),(V1,V2))| R]):-
        getRank(L1,M,1,Rank1),
        getRank(L2,M,1,Rank2),
        invalidPairMechanismDetection(R2,M,R).

getRank(L1,[LA|R],Input,Input):-eg(L1,LA),!.
getRank(L1,[LA|R],Input,Rank):-
        Input2 is Input + 1,
        getRank(L1,R,Input2,Rank).

eg([],[]).
eg([VA|RA],[VB|RB]):-
        VA == VB,
        eg(RA,RB).


/*
discardinvalidPair(InvalPair, M,PWCtr, PWCtrLight).
*/


discardinvalidPair([],_, _, []).

discardinvalidPair([A|R], M,PWCtr, PWCtrLight2):-
        discardinvalidPair2(A,M,PWCtr,PWCtrLight),
        discardinvalidPair(R, M,PWCtrLight, PWCtrLight2).


discardinvalidPair2(_,_, [], []).

discardinvalidPair2(InvalPair,M, [A|R], R2):-
      isinvalid(InvalPair,A,M),!,
      discardinvalidPair2(InvalPair,M, R, R2).

discardinvalidPair2(InvalPair,M,[A|R], [A|R2]):-
      discardinvalidPair2(InvalPair,M, R,R2).

isinvalid([],[],M).

isinvalid([b((R1,R2),(V1,V2))|_],pairwise(I,(L1,L2),(V1,V2)),M):-
      getRank(L1,M,1,R1),
      getRank(L2,M,1,R2),!.
            
isinvalid([b((_,_),(_,_))|R],pairwise(I,(L1,L2),(V1,V2)),M):-
      isinvalid(R,pairwise(I,(L1,L2),(V1,V2)),M).
            
unittest:-
        M = [[A1,A2,A3],[B1,B2,B3],[C1,C2,C3]],
        Ctr = [pairwise(1,([A1,A2,A3],[B1,B2,B3]),(0,0)),pairwise(3,([A1,A2,A3],[B1,B2,B3]),(0,0))],
        invalidPairDetectionMechanism(Ctr,2,Ctr2),
        writeln(Ctr2),
        invalidPairMechanismDetection(Ctr2,M,RR),
        M2 = [[AA1,AA2,AA3],[BB1,BB2,BB3]],
        Ctr3 = [pairwise(_,([AA1,AA2,AA3],[BB1,BB2,BB3]),(0,0)),pairwise(_,([AA1,AA2,AA3],[BB1,BB2,BB3]),(0,0))],
        discardinvalidPair(RR,M2, Ctr3, Ctr32),
        writeln(Ctr32).

/* test 
        
          M2 = [[AA1,AA2,AA3],[BB1,BB2,BB3]],
        Ctr2 = [pairwise(_,([AA1,AA2,AA3],[BB1,BB2,BB3]),(0,0)),pairwise(_,([AA1,AA2,AA3],[BB1,BB2,BB3]),(0,0))],
        writeln(RR),
        discardinvalidPair(RR,M, Ctr2, Ctr32),
        writeln(Ctr32).*/

inval2:-
        Features = [R,A,B,A1,A2,B1,B2,B3],CtrList = [R = 1,and(R,[A,B]),xor(A,[A1,A2]), xor(B,[B1,B2,B3]),
        cnf([A1],[B1]),cnf([A2],[B2,B3])],               
        borneRec(Features),
        assertRec([a(_,_,[null,0,1])]),  
        put_Attr_rec(Features) ,        
        callRec(CtrList),                   
        length(Features,FeaturesNumber),
        matrice(M,FeaturesNumber,5),
        limitMatrix(M,Features),
        writeln(' Pairs generation... '),!,
        pairwiseGenerator(Features,M,PWCTRLST,RANKLST,CtrList), 
        callRec(PWCTRLST), 
        flatten(RANKLST,HLVL_I),     
        getLabelVar(HLVL_I,I_RANK),           
        contrainteFDV2(M,Features,CtrList,Res,4),
        callRec(Res),
        domain(I_RANK,1,20),
        (I_RANK \= [] ->maximum(Max,I_RANK)),!,
        length(PWCTRLST,Li),
        write('Nb paires : '), writeln(Li),
        writeln(' Solving... '),  
        maximum(K,I_RANK),!,
        labeling([ff],I_RANK),!,
        write('Max :'), writeln(K),
        printlst(PWCTRLST),
        invalidPairDetectionMechanism(PWCTRLST,2,R2),
        invalidPairMechanismDetection(R2,M,RR),
        writeln(RR),      
        Features2 = [R,A,B,A1,A2,B1,B2,B3],CtrList2= [R = 1,and(R,[A,B]),xor(A,[A1,A2]), xor(B,[B1,B2,B3]),
        cnf([A1],[B1]),cnf([A2],[B2,B3])],
        borneRec(Features2),
        assertRec([a(_,_,[null,0,1])]),  
        put_Attr_rec(Features2) ,   
        callRec(CtrList2),                   
        length(Features2,FeaturesNumber),
        matrice(M2,FeaturesNumber,10),
        limitMatrix(M2,Features),
        writeln(' Pairs generation... '),!,
        pairwiseGenerator(Features2,M2,PWCTRLST2,RANKLST2,CtrList2), 
        writeln(PWCTRLST2),
        length(PWCTRLST2,Li2),
        write('Nb paires : '), writeln(Li2),
        printlst(PWCTRLST22),
        discardinvalidPair(RR, M2,PWCTRLST2, PWCTRLST22),
        length(PWCTRLST22,Li22),
        printlst(PWCTRLST22),
        write('Nb paires : '), writeln(Li22).
                                                          

invalDetectTest:-
        Features = [A,B,C,D,E],
         borneRec(Features),
       assertRec([a(_,_,[null,0,1])]),  
         put_Attr_rec(Features) ,
         CtrList = [xor(A,[B,C,D,E])],
        callRec(CtrList),                   
        length(Features,FeaturesNumber),
        matrice(M,FeaturesNumber,20),
        limitMatrix(M,Features),
        writeln(' Pairs generation... '),!,
        pairwiseGenerator(Features,M,PWCTRLST,RANKLST,CtrList), 
        callRec(PWCTRLST), 
        flatten(RANKLST,HLVL_I),     
         getLabelVar(HLVL_I,I_RANK),           
        contrainteFDV2(M,Features,CtrList,Res,4),
        callRec(Res),
        domain(I_RANK,1,20),
        
        (I_RANK \= [] ->maximum(Max,I_RANK)),!,
        length(I_RANK,Li),
        write('Nb paires : '), writeln(Li),
        writeln(' Solving... '),  
        maximum(K,I_RANK),
     
        labeling([ff],I_RANK),
        write('Max :'), writeln(K),
        writeln(PWCTRLST),
        invalidPairDetectionMechanism(PWCTRLST,16,R2),
        invalidPairMechanismDetection(R2,M,RR),
        writeln(RR),
           Features2 = [A2,B2,C2,D2,E2],
         borneRec(Features2),
       assertRec([a(_,_,[null,0,1])]),  
         put_Attr_rec(Features2) ,
         CtrList2 = [xor(A2,[B2,C2,D2,E2])],
        callRec(CtrList2),                   
        length(Features2,FeaturesNumber),
        matrice(M2,FeaturesNumber,10),
        limitMatrix(M2,Features),
        writeln(' Pairs generation... '),!,
        pairwiseGenerator(Features2,M2,PWCTRLST2,RANKLST2,CtrList2), 
        
       writeln(PWCTRLST2),
        discardinvalidPair(RR, M2,PWCTRLST2, PWCTRLST22),
        writeln(PWCTRLST22).

buginvest:-
        Features= [AR,AR1,AR2,AR4,AR12,AR1617,AR1618,AR156,AR157,AR289,AR2810,AR2811,AR121314,AR121315],
       CtrList = [and(AR,[AR1,AR2,AR12]),opt(AR,[AR4]),or(AR,[AR1617,AR1618]),xor(AR1,[AR156,AR157]),xor(AR2,[AR289,AR2810,AR2811]),xor(AR12,[AR121314,AR121315]),cnf([],[AR4, AR2810]) ,cnf([],[AR2811, AR121315]) ,AR= 1],
         borneRec(Features),
       assertRec([a(_,_,[null,0,1])]),  
         put_Attr_rec(Features) ,
        callRec(CtrList),                   
        length(Features,FeaturesNumber),
        matrice(M,FeaturesNumber,10),
        limitMatrix(M,Features),
        writeln(' Pairs generation... '),!,
        pairwiseGenerator(Features,M,PWCTRLST,RANKLST,CtrList), 
        printlst(PWCTRLST),
        printlst(RANKLST),
         getLabelVar(HLVL_I,I_RANK),           
        contrainteFDV2(M,Features,CtrList,Res,4),
        callRec(Res),
        domain(I_RANK,1,20),
        
        (I_RANK \= [] ->maximum(Max,I_RANK)),!,
                length(PWCTRLST,LiP),
        write('Nb paires : '), writeln(LiP),
        length(I_RANK,Li),
        write('Nb paires : '), writeln(Li).

getI([],[]).
getI([pairwise(I,(_,_),(_,_),(0,0))|R],[I|R2]):-
        getI(R,R2).


validconfig:-openfile(_,_,Features,CtrList,_,_,_,_),
             borneRec(Features),
             callRec(CtrList),
             labeling([],Features).
             

nbconfig:-openfile(_,_,Features,CtrList,_,_,_,_),
        assert((t(0))),
        open('nbConfig.txt', append, Stream),write(Stream,_),write(Stream,';'),close(Stream),
        allconfig(Features,Features, CtrList),
        retract((t(L))),
        open('nbConfig.txt', append, Stream2),write(Stream2,L),nl(Stream2),close(Stream2),
        write(' L : '),writeln(L), halt.
        

lab(FeaturesList) :-      
        labeling([],FeaturesList),
        retract((t(N))),
        N1 is N+1,
        asserta((t(N1))).

allconfig(FeaturesList,_, Constraints) :-
        domain(FeaturesList,0,1),
        callRec(Constraints),
        findall(_,lab(FeaturesList),_).

runJournal:- solverGen([a(_,_,[null,0,1])],[],[CAR,ADC,CAB,SA,EA,PP,BS,LRF,FF],[and(CAR,[ADC,CAB]),or(SA,[EA,PP]),CAR = 1],10,'CARFd',[ff],_,yes,yes,yes,'car.xml').


vcs:-  solverGen([a(_,_,[null,0,1])],[],[VCS,CALL,P2P,MULTISITE,A,B,C,PREMIUM_RESOLUTION,D,E],[and(VCS,[CALL]),opt(VCS,[PREMIUM_RESOLUTION]),xor(CALL,[P2P,MULTISITE]), xor(MULTISITE,[A,B,C]),
       or(PREMIUM_RESOLUTION,[D,E]), require(B,PREMIUM_RESOLUTION),require(C,PREMIUM_RESOLUTION),VCS = 1],30,'CARFd',[ff],_,yes,yes,yes,'car.xml').



run0:-openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,FileName) ,
                                 solverGen(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,[ff],yes,FileName).


     openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName) :-
        open('model.txt', read, Stream),
        read(Stream,[PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName]),close(Stream).                         



reason(Features,CtrList):-
        borneRec(Features), 
        callRec(CtrList).

validPairNumber(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,_,_):-        
        writeStat0(ModelName,'./stats.txt'),
        writeln(' Loading model... '),
        writeln(Features),
        borneRec(Features),
        statistics(runtime, [T0|_]),
        assertRec(PairInformations),  
        (Attributes = [] -> put_Attr_rec(Features) ;       callRec(Attributes)),
        callRec(CtrList),                   
        length(Features,FeaturesNumber),
        matrice(M,FeaturesNumber,MatrixSize),
        limitMatrix(M,Features),
        writeln(' Pairs generation... '),
        pairwiseGenerator(Features,M,PWCTRLST,_,Features), 
        callRec(PWCTRLST),   
        length(PWCTRLST,NbValidPair),      
        N  is(FeaturesNumber * (FeaturesNumber - 1)  * 2),
        open('./stats.txt', append, Stream),write(Stream,NbValidPair),write(Stream,';'),write(Stream,N),close(Stream),
        halt.

writelnFile([],F).
writelnFile([A|R],Stream):-write(Stream,A),nl(Stream),writelnFile(R,Stream).


nbconfig:-openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization),
        assert((t(0))),
        open('nbConfig.txt', append, Stream),write(Stream,Etiquette),write(Stream,';'),close(Stream),
        allconfig(Features,Features, CtrList),
        retract((t(L))),
        open('nbConfig.txt', append, Stream2),write(Stream2,L),nl(Stream2),close(Stream2),
        write(' L : '),writeln(L), halt.



lab(FeaturesList) :-      
        labeling([],FeaturesList),
        retract((t(N))),
        N1 is N+1,
        asserta((t(N1))).

  allconfig(FeaturesList,PrimitivesFeaturesList, Constraints) :-
         domain(FeaturesList,0,1),
          callRec(Constraints),
      findall(_,lab(FeaturesList),_).
/*
minimize(K),time_out(100000,_*/

writeFt(Features):-      open('./Feature.txt', append, Stream),nl(Stream),writelnFile(Features,Stream),nl(Stream),close(Stream).
writeCtr(Ctr):-open('./ctr.txt', append, Stream2),nl(Stream2),writelnFile(Ctr,Stream2),nl(Stream2),close(Stream2).  

invalidModel:-
                L = [R,A,B,A1,A2,B1,B2,B3],domain(L,0,1), Ctr = [R = 1,and(R,[A,B]),xor(A,[A1,A2]), xor(B,[B1,B2,B3]),
                cnf([A1],[B1]),cnf([A2],[B2,B3])],solverGen([a(_,_,[null,0,0])],[],L,Ctr,10,'test',[ff],_,yes,yes,yes,'test.txt').


solverGen(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName,T,R2,InvalidPairs):-   
        introspection(Labelling,Minimization,AllDif,Sort,SymetricBreaking), 
        writeStat0(FileName,'./stats.txt'),
        writeln(' Loading model... '),     
        borneRec(Features),
        statistics(runtime, [T0|_]),
        assertRec(PairInformations),  
        (Attributes = [] -> put_Attr_rec(Features) ;       callRec(Attributes)),
        callRec(CtrList),     
        callRec(CtrList), 
        callRec(CtrList),              
        writeln(Features),
        length(Features,FeaturesNumber),
        matrice(M,FeaturesNumber,MatrixSize),
        limitMatrix(M,Features),
        writeln(' Pairs generation... '),!,
        pairwiseGenerator(Features,M,PWCTRLST,RANKLST,CtrList),
        callRec(PWCTRLST), 
        length(PWCTRLST,Li),
        write('Nb paires : '), writeln(Li),
        (PWCTRLST \= [] ->
        (AllDif = 'yes' -> writeln('All Diff'),alldiffrec(RANKLST) ; write('')),     
        flatten(RANKLST,HLVL_I),          
        (Sort = 'yes' -> my_sort(HLVL_I,HLVL_I_Sort),getLabelVar(HLVL_I_Sort,I_RANK) ; getLabelVar(HLVL_I,I_RANK) ),           
        contrainteFDV2(M,Features,CtrList,Res,0),
        callRec(Res),
        domain(I_RANK,1,MatrixSize),
        (I_RANK \= [] ->maximum(Max,I_RANK)),!,
        length(I_RANK,LI_RANK),
        write('Nb paires : '), writeln(LI_RANK),
        writeln(' Solving... '),  
        maximum(K,I_RANK),
       (number(Minimization) -> Label = [minimize(K)|Labelling] ; Label = Labelling),
        labeling(Label,I_RANK),!,
        statistics(runtime, [T1|_]),
        T is T1 - T0,
        detectInvalidPairRank(I_RANK,R2),
        write(' Solution found in : '), write(T),writeln(' ms'),
        write(' Number of configurations : '), writeln(Max),
        nvariable(M,L,Max),
        labeling([ff],L),
        submatrix(M,M2,K),
        fd_statistics(resumptions,Resumptions),
        fd_statistics(entailments,Entailments),  
        fd_statistics(prunings,Prunings),  
        fd_statistics(backtracks,Backtracks),  
        fd_statistics(constraints,Constraints),  
        writeStatV3(FeaturesNumber,T,Max,R2,'./stats.txt',Labelling,Sort,Resumptions,Entailments,Prunings,Backtracks,Constraints,ModelName),
        writeMat2(ModelName,M2))
        .

test22:-        openfile(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName),
         solverGen2(PairInformations,Attributes,Features,CtrList,16,ModelName,Labelling,70000,AllDif,Sort,SymetricBreaking,FileName,[]).

solverGen2(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization,AllDif,Sort,SymetricBreaking,FileName,InvalidPairs):-   
        introspection(Labelling,Minimization,AllDif,Sort,SymetricBreaking), 
        MZ2 is MatrixSize ,
        MiTime is Minimization * 4 ,
        writeStat0(FileName,'./stats2.txt'),
        writeln(' Loading model... '),     
        borneRec(Features),
        statistics(runtime, [T0|_]),
        assertRec(PairInformations),  
        (Attributes = [] -> put_Attr_rec(Features) ;       callRec(Attributes)),
        callRec(CtrList),     
        callRec(CtrList), 
        callRec(CtrList),               
        length(Features,FeaturesNumber),
        matrice(M,FeaturesNumber,MZ2),
        limitMatrix(M,Features),
        writeln(' Pairs generation... '),!,
        pairwiseGenerator(Features,M,PWCTRLST1,RANKLST,CtrList), 
        length(PWCTRLST1,L1),
        write('before detection:'),
        writeln(L1),       
        discardinvalidPair2(InvalidPairs, M,PWCTRLST1, PWCTRLST),
        length(PWCTRLST,L2),
        write('after detection:'),
        writeln(L2),
        callRec(PWCTRLST),   
        (PWCTRLST \= [] ->
        (AllDif = 'yes' -> writeln('All Diff'),alldiffrec(RANKLST) ; write('')), 
        flatten(RANKLST,HLVL_I),     
        (Sort = 'yes' -> my_sort(HLVL_I,HLVL_I_Sort),getLabelVar(HLVL_I_Sort,I_RANK) ; getLabelVar(HLVL_I,I_RANK) ),           
        contrainteFDV2(M,Features,CtrList,Res,0),
        callRec(Res),
        domain(I_RANK,1,MZ2),
        
        (I_RANK \= [] ->maximum(Max,I_RANK)),!,
        length(I_RANK,Li),
        write('Nb paires : '), writeln(Li),
        writeln(' Solving... '),  
        
        /*maximum(K,I_RANK),*/
        sum(I_RANK,#=,K), 
       (number(MiTime) -> Label = [minimize(K),time_out(MiTime,_)|Labelling] ; Label = Labelling),!,
        labeling(Label,I_RANK),
         statistics(runtime, [T1|_]),
        T is T1 - T0,
        detectInvalidPairRank(I_RANK,R2),
        write(' Solution found in : '), write(T),writeln(' ms'),
        write(' Number of configurations : '), writeln(Max),
        nvariable(M,L,Max),
        labeling([ff],L),
        submatrix(M,M2,K),
        fd_statistics(resumptions,Resumptions),
        fd_statistics(entailments,Entailments),  
        fd_statistics(prunings,Prunings),  
        fd_statistics(backtracks,Backtracks),  
        fd_statistics(constraints,Constraints),  
        writeStatV3(FeaturesNumber,T,Max,R2,'./stats2.txt',Labelling,Sort,Resumptions,Entailments,Prunings,Backtracks,Constraints,ModelName),
        writeMat2(ModelName,M2)).


writeStatV2(Nbft,Temps,NbConfig,NBVraiConf, File,Labeling, Sort,Resumptions,Entailments,Prunings,Backtracks,Constraints):-
        open(File, append, Stream),
        write(Stream,Nbft),
        write(Stream,';'),
        write(Stream,Temps),
        write(Stream,';'),
        write(Stream,NbConfig),
        write(Stream,';'),
        write(Stream,NBVraiConf),
        write(Stream,';'),
        write(Stream,Labeling),
        write(Stream,';'),
        write(Stream,Sort),
        write(Stream,';'),
        write(Stream,Resumptions),
        write(Stream,';'),
        write(Stream,Entailments),
        write(Stream,';'),
        write(Stream,Prunings),
        write(Stream,';'),
        write(Stream,Backtracks),
        write(Stream,';'),
        write(Stream,Constraints),
        nl(Stream),
        close(Stream).

writeStatV3(Nbft,Temps,NbConfig,NBVraiConf, File,Labeling, Sort,Resumptions,Entailments,Prunings,Backtracks,Constraints,FileName):-
        open(File, append, Stream),
        write(Stream,Nbft),
        write(Stream,';'),
        write(Stream,Temps),
        write(Stream,';'),
        write(Stream,NBVraiConf),
        write(Stream,';'),
        write(Stream,Labeling),
        write(Stream,';'),
        write(Stream,Sort),
        write(Stream,';'),
        write(Stream,Resumptions),
        write(Stream,';'),
        write(Stream,Entailments),
        write(Stream,';'),
        write(Stream,Prunings),
        write(Stream,';'),
        write(Stream,Backtracks),
        write(Stream,';'),
        write(Stream,Constraints),
        write(Stream,';'),
        write(Stream,FileName),
        nl(Stream),        
        close(Stream).
solver(PairInformations,Attributes,Features,CtrList,MatrixSize,ModelName,Labelling,Minimization):-
        writeStat0(ModelName,'./stats.txt'),
         writeln(' Loading model... '),
         writeln(Features),
         borneRec(Features),
        statistics(runtime, [T0|_]),
        assertRec(PairInformations),  
        (Attributes = [] -> put_Attr_rec(Features) ;       callRec(Attributes)),
        callRec(CtrList),                   
        length(Features,FeaturesNumber),
        matrice(M,FeaturesNumber,MatrixSize),
        limitMatrix(M,Features),
        writeln(' Pairs generation... '),
        pairwiseGenerator(Features,M,PWCTRLST,RANKLST,Features), 
        callRec(PWCTRLST),   
   
        alldiffrec(RANKLST),
        flatten(RANKLST,HLVL_I),  
  /*         my_sort(HLVL_I,HLVL_I_Sort),
        getLabelVar(HLVL_I_Sort,I_RANK),*/
           getLabelVar(HLVL_I,I_RANK),
       contrainteFDV2(M,Features,CtrList,Res,4),

        callRec(Res),
        domain(I_RANK,1,MatrixSize),
        (I_RANK \= [] ->maximum(Max,I_RANK)),!,
        writeln(' Solving... '),
  
        maximum(K,I_RANK),
     (   Minimization = 0 ->         labeling([ff],I_RANK)  ; labeling([ff,time_out(Minimization,_),minimize(K)],I_RANK) ),

        length(I_RANK,LL),
        writeln(LL),
        statistics(runtime, [T1|_]),
        detectInvalidPairRank(I_RANK,R2),
        writeln(I_RANK),
        T is T1 - T0,
        write(' Solution found in : '), write(T),writeln(' ms'),
        write(' Number of configurations : '), writeln(Max),
        nvariable(M,L,Max),
        labeling([ff],L),
        submatrix(M,M2,Max),
        writeStat(FeaturesNumber,T,Max,R2,'./stats.txt'),
        writeMat2('./matrix.txt',M2),       
        halt.

 


pwMonitor([]).
pwMonitor([[]|R]):-!,pwMonitor(R).
pwMonitor([A|R]):-monitoring2(A),pwMonitor(R).
/*

   (number(Minimization) ->  labeling([ff,minimize(Max),time_out(Minimization,_)],I_RANK);   */

save:-save_program('/home/aymeric/These/Pacogen/tools/splot_to_Model/models/Pacogen.sav').


save2:-save_program('/home/aymeric/Bureau/Collab_Mathieu/Pacogen.sav').


save3:-save_program('/home/aymeric/These/Pacogen/Eval/Selected Models/Pacogen.sav').
alldiffrec([]).
alldiffrec([A|R]) :-
                     getLabelVar(A,AA),
                     all_different(AA),
                     alldiffrec(R). 



alldiffrec2([]).

alldiffrec2([A|R]) :- 
                   
                     all_different(A),
                     alldiffrec2(R). 

/*
GlobalConstraint
*/


        clpfd:dispatch_global(and(P,LF), state(_), state(_), Actions) :-
            and_solver(P,  LF,Actions).  

        clpfd:dispatch_global(or(P,_), state(LF0), state(LF), Actions) :-
            or_solver(P, LF0, LF,Actions).    

        clpfd:dispatch_global(xor(P,_), state(LF0), state(LF), Actions) :-
            xor_solver(P, LF0, LF,Actions). 

        clpfd:dispatch_global(mand(P,LF), state(_), state(_), Actions) :-
            mand_solver(P,  LF,Actions).

        clpfd:dispatch_global(opt(P,LF), state(_), state(_), Actions) :-
             opt_solver(P,  LF,Actions).

        clpfd:dispatch_global(card(Min,Max,P,LF),state(_),state(_),Actions):-
             card_solver(Min,Max,P,LF,Actions).

        clpfd:dispatch_global(require(A,B), state(_), state(_), Actions) :-
            require_solver(A, B,Actions).

        clpfd:dispatch_global(mutex(A,B), state(_), state(_), Actions) :-
            mutex_solver(A,B,Actions).

        clpfd:dispatch_global(cnf(A,B), state(_), state(_), Actions) :-
            cnf_solver(A,B,Actions).
                        
        clpfd:dispatch_global(monitoring(K,_,T),state(I0),state(I),Actions) :-
                monitoring_solver(K,I0,I,T,Actions).

        clpfd:dispatch_global(monitoring2(A),state(_),state(_),Actions) :-
                monitoring_solverRes(A,Actions).

        clpfd:dispatch_global(pw(I,L1,L2,V1,V2), state, state, Actions) :-
                pw_solver(I,L1,L2,V1,V2,Actions).

        clpfd:dispatch_global(monitorMax(M,T,F), state(_), state(_), Actions) :-
                monitorMax_sovler(M,T,F,Actions).



/*
monitorMax(M,T,File):-
        
      fd_max(M,Mx),
      assert(m(Mx)),
        fd_global(monitorMax(M,T,File), state(Mx), [max(M)]).

      monitorMax_sovler(M,T0,File,Actions):-
            retract(m(S1)),
              
              ( number(M),M < S1 -> assert(m(M)), 
               statistics(runtime, [T1|_]),
                 T is T1 - T0 ,
                     open('./resMonitor.txt', append, Stream),write(Stream,File),write(Stream,';'),write(Stream,M),write(Stream,';'),write(Stream,T),nl(Stream),close(Stream),
            Actions = []    ; assert(m(S1)), Actions = [])    
            .*/


monitorMax(M,T,File):-

        fd_global(monitorMax(M,T,File), state(Mx), [max(M)]).

      monitorMax_sovler(M,T0,File,Actions):-
            
      fd_max(M,Mx),
    
              
         
               statistics(runtime, [T1|_]),
                 T is T1 - T0 ,
                     open('./resMonitor.txt', append, Stream),write(Stream,File),write(Stream,';'),write(Stream,Mx),write(Stream,';'),write(Stream,T),nl(Stream),close(Stream),
            Actions = []   .


      /*  clpfd:dispatch_global(monitoringPaire(Val1,Val2,Index,L1,L2),state(_),state(_),Actions) :-
                monitoringPaire_solver(Val1,Val2,Index,L1,L2,Actions).*/


        setVal(_,[],[]).
        setVal(Val,[A|R],[A = Val|R2]):-setVal(Val,R,R2).

monitoring2(A):-
                A = pairwise(I,(CA,CB),(VA,VB)),
                fd_global(monitoring2(A), state(I), [dom(I)]).


              monitoring_solverRes(A,Actions):-
                   A = pairwise(I,(CA,CB),(VA,VB)),
                           open('./pairsMonitor.txt', append, Stream),write(Stream,A),write(Stream,'.'),nl(Stream),close(Stream),
                             open('./pairs.txt', append, Stream3),write(Stream3,I),write(Stream3,'.'),nl(Stream3),close(Stream3),Actions = [exit].
       





        cnf(A,B):- 
                dom_suspensions(A,Susp),
                dom_suspensions(B,Susp2),
                append(Susp,Susp2,SuspS),
                fd_global(cnf(A,B), state(_), SuspS).

/*
Compte le nolmbre d'occurence de -1 et de 0, rend la liste des variables non instanciées
       N : nbre de valeur supérieure à 0
       M : nbre de 0
                    filter(L,LFLibre,N,M)
*/
/*
A : neg
B : not neg   
-*/

        cnf_solver(A,B,Actions):-
               filter(A,A_Free,NA,MA),
               filter(B,B_Free,NB,MB),
               length(A,LA),
               length(B,LB),
    
               (
                  /* cases where B = []) */
                  LB == 0, MA > 0 -> Actions =  [exit] ;
                  LB == 0,NA is LA - 1 -> setVal(0,A_Free,R),Actions =  [exit| R] ;  
                  
                  MB \= 0,MB == LB ->  setVal(0,A,R),Actions =  [exit| R] ;
                  MA > 0 -> Actions = [exit] ;
                  NB > 0 -> Actions = [exit] ;
                  NA == LA, MB is LB - 1 -> setVal(1,B_Free,R),Actions =  [exit| R] ;
                  MB == LB, NA is LA - 1 -> setVal(0,A_Free,R),Actions =  [exit| R] ;
                  Actions = []
               
                  ).

                  
               
                  require_solver(A, B,Actions) :-
               (
                B == 0 -> Actions =  [exit, A = 0];   
                B == 1 -> Actions =  [exit];     
                A == 0 -> Actions =  [exit];   
                A == 1 -> Actions =  [exit, B = 1];       
                Actions = []
              ).      
               
               
               



        and(P,LF):-
              dom_suspensions(LF,Susp),Susp2 = [val(P),min(P)|Susp],
              fd_global(and(P,LF), state(_), Susp2).

  

         and_solver(P,LF,Actions) :-
                filter(LF,_,N,M),

             ( M > 0                   -> Actions = [exit |Ps], ex_eq([P |LF],0,Ps)
             ; P == 0                 -> Actions = [exit |Ps], ex_eq(LF,0,Ps)
             ; fd_min(P,K), K > 0     -> Actions = [exit |Ps],ex_neq(LF,0,Ps)
             ; N > 0                   -> Actions = [exit |Ps], ex_neq([P |LF],0,Ps)
             ; Actions = []
             ).
          



        or(P,LF):-       
              dom_suspensions(LF,Susp),Susp2 = [min(P),val(P)|Susp],
              fd_global(or(P,LF), state(LF), Susp2).

        
         or_solver(P, LF0, LF,Actions) :-
               filter(LF0,LF,N,M),
               length(LF0,L),/* N : nbre de 1 dans la liste */
         
             ( P == 0                                              -> Actions = [exit |Ps],ex_eq(LF0,0,Ps)
             ; fd_min(P,K), K > 0, N > 1                           -> Actions = [exit]
             ; fd_min(P,K), K > 0,L2 is  L - 1 ,M == L2, N==0      -> LF = [A],Actions = [exit ,A = 1]
             ; N > 0                                                -> Actions = [exit |Ps1],ex_neq([P], 0, Ps1)
             ; M == L                                               -> Actions = [exit |Ps1],ex_eq([P], 0, Ps1)
             ; Actions = []
             ).
          

      

        xor(P,LF):-
              dom_suspensions(LF,Susp),Susp2 = [min(P),val(P)|Susp],
              fd_global(xor(P,LF), state(LF), Susp2).
         xor_solver(P, LF0, LF,Actions) :-
               filter(LF0,LF,N,M) ,
                 length(LF0,L),/* N : nbre de 0 dans la liste , M nbre de - 1*/
           
             ( P == 0                                  -> Actions = [exit |Ps], ex_eq(LF0,0,Ps)
             ; N > 1                                    -> Actions = [fail]
             ; N == 1                                   -> Actions = [exit |Ps3],ex_neq([P], 0, Ps2), ex_eq(LF, 0, Ps),append(Ps,Ps2,Ps3) /* tous les éléments de la liste sont égaux à -1 */
             ; L2 is L-1, M == L2, fd_min(P,K), K > 0  -> Actions = [exit |Ps3],ex_neq([P], 0, Ps2), ex_neq(LF, 0, Ps),append(Ps,Ps2,Ps3) 
             ; M == L                                   -> Actions = [exit |Ps1], fdset_singleton(Set, 0),Ps1 = [P in_set Set]
             ; Actions = []
             ).





       mand(P,LF):-
              dom_suspensions(LF,Susp),Susp2 = [min(P),val(P)|Susp],
              fd_global(mand(P,LF), state(_), Susp2).

        mand_solver(P, LF,Actions) :-
                filter(LF,_,N,M),

             ( M > 0                   -> Actions = [exit |Ps], ex_eq([P |LF],0,Ps)
             ; P == 0                 -> Actions = [exit |Ps], ex_eq(LF,0,Ps)
             ; fd_min(P,K), K > -1     -> Actions = [exit |Ps],ex_neq(LF,0,Ps)
             ; N > 0                   -> Actions = [exit |Ps], ex_neq([P |LF],0,Ps)
             ; Actions = []
             ).


        mutex(A,B):-fd_global(mutex(A,B), state(_), [dom(A),dom(B)]).
        
        mutex_solver(A,B, Actions) :-
                (
                A == 1 ->Actions = [exit, call(B = 0)] ;
                A == 0 ->Actions = [exit] ;
                B == 1 ->Actions = [exit, call(A = 1)] ;
                B == 0 ->Actions = [exit] ;
                Actions = []
                ).


       opt(P,LF):-
              dom_suspensions(LF,Susp),Susp2 = [val(P), min(P)|Susp],
              fd_global(opt(P,LF), state(_), Susp2).

        opt_solver(P, LF,Actions) :-       
              filter(LF,_,N,_),        
             ( N > 0                 ->  Actions = [exit|Ps], ex_neq([P],0,Ps)
             ; P == 0               ->  Actions = [exit|Ps], ex_eq(LF,0,Ps)
             ; fd_min(P,K), K > 0   ->  Actions = [exit]
             ; Actions = []
             ).



       

        card(Min,Max,P,LF):-
             dom_suspensions(LF,Susp),Susp2 = [val(P)|Susp],
             fd_global(card(Min,Max,P,LF), state(LF), Susp2).

        card_solver(Min,Max,P, LF,Actions) :-
                filter(LF,LF2,N,_),
                (
                 P == 0                           -> Actions = [exit|Ps], ex_eq(LF,0,Ps);
                 P == 1, N > Max                  -> Actions = [fail];
                 P == 1, N < Min, D is Min - N,length(LF2,A), D > A 
                                                  -> Actions = [fail];
                 P == 1, N == Max                 -> Actions = [exit|Ps], ex_eq(LF2,0,Ps);
                 P == 1, N < Min, D is Min - N,length(LF2,D) 
                                                  -> Actions = [exit|Ps], ex_eq(LF2,1,Ps);
                 Actions = []
                 ).



        

         require(A,B):-
             fd_global(require(A,B), state(_), [dom(A),dom(B)]).
/*
A => B
*/






      

        copy([],[]).
        copy([A|R],[A|R2]):- 
                copy(R,R2).

      dom_suspensions([], []).
     dom_suspensions([X|Xs], [dom(X)|Susp]) :-
             dom_suspensions(Xs, Susp).




                        

/*
Permet de visualiser le nombre de paires inscrites dans la matrice
*/
        monitoring(K,I,T):-
                dom_suspensionsDomains(I,Susp,[]),
                fd_global(monitoring(K,I,T), state(I), Susp).

        monitoring_solver(K,I0,I,T,Actions):-
                   statistics(runtime, [T1|_]),
                   T0 is T1 - T,
                        monitoring_filter(I0,I2,N),
              length(I0,Li),
                   copy(I2,I),
         open('monitor.txt', append, Stream),nl(Stream),write(Stream,K),write(Stream,';'),write(Stream,Li),write(Stream,';'),write(Stream,T0),nl(Stream),close(Stream),
                 (  N == 0 -> Actions = [exit]
                  ;Actions = []).


        monitoring_filter([A|R],I2,N):- 
               number(A), 
               ! ,
               monitoring_filter(R,I2,N).

        monitoring_filter([A|R],[A|I2],N2) :- 
               monitoring_filter(R,I2,N),
               N2 is N +1 .
        
        monitoring_filter([],[],0).


        lineConstraint(Li,Ctr,FT):-
                dom_suspensionsDomains(Li,Susp,[]),
                fd_global(lineConstraint(Li,Ctr,FT), state(Li), Susp).
        

        cpDomain([],[],[]).     

        cpDomain([Li0|Li0R],[Li|LiR],[Li in_set Set|Ps0]) :-
                fd_set(Li0,Set), 
                cpDomain(Li0R,LiR,Ps0).

 

        ligne_Filter([A|B],I) :- 
                number(A),
                !,
                ligne_Filter(B,I).
        
        ligne_Filter([_|B],I) :- 
                ligne_Filter(B,I1),
                I is I1 +1.
        
        ligne_Filter([],0).



     dom_suspensionsDomains([], Tail,Tail).

     dom_suspensionsDomains([X|Xs], [dom(X)|Susp],Tail) :-
             dom_suspensionsDomains(Xs, Susp,Tail).

     dom_suspensionsValues([], Tail,Tail).
     dom_suspensionsValues([X|Xs], [val(X)|Susp],Tail) :-
             dom_suspensionsValues(Xs, Susp,Tail).






   








   





/*
Impose la présence d'une valeur supérieur à 0
*/
       or_eq([_|LFr],Ps) :-
                 or_eq(LFr,Ps).
               
       or_eq([LF|LFr],[LF in_set Set|Ps]) :-
                fdset_singleton(Set0, 0),
                fdset_complement(Set0, Set),
                or_eq2(LFr,Ps).
                                  
       or_eq2(_,[]).        

        filter([],[],0,0).
/*
Compte le nolmbre d'occurence de -1 et de 0, rend la liste des variables non instanciées
       N : nbre de valeur supérieure à -1
       M : nbre de -1
                    filter(L,LFLibre,N,M)
*/
        filter([LF|LFs],LF0,N,M) :- 
                 fd_min(LF,K), K > 0 ,!,
                 filter(LFs,LF0,N1,M), N is N1 +1 .
               
     
        filter([LF|LFs],LF0,N,M) :-  
                LF == 0,!,
               filter(LFs,LF0,N,M1), M is M1 +1 .

        filter([LF|LFs],LF0,N,M):-
                LF0 = [LF|LF02],
                filter(LFs,LF02,N,M).



                

                       


% exactly.pl
     

     % rules [1,2]: filter the X's, decrementing N
     ex_filter([], [], N, N, _).
     ex_filter([X|Xs], Ys, L, N, I) :- X==I, !,
             M is L-1,
             ex_filter(Xs, Ys, M, N, I).
     ex_filter([X|Xs], Ys0, L, N, I) :-
             fd_set(X, Set),
             fdset_member(I, Set), !,
             Ys0 = [X|Ys],
             ex_filter(Xs, Ys, L, N, I).
     ex_filter([_|Xs], Ys, L, N, I) :-
             ex_filter(Xs, Ys, L, N, I).
     
     % rule [3]: all must be neq I
     ex_neq(Xs, I, Ps) :-
             fdset_singleton(Set0, I),
             fdset_complement(Set0, Set),
             eq_all(Xs, Set, Ps).
     
     % rule [4]: all must be eq I
     ex_eq(Xs, I, Ps) :-
             fdset_singleton(Set, I),
             eq_all(Xs, Set, Ps).
     
     eq_all([], _, []).
     eq_all([X|Xs], Set, [X in_set Set|Ps]) :-
             eq_all(Xs, Set, Ps).

      contrainte(A,B,L1,L2,R)  :-
             dom_suspensions2(L1,L2, Susp),
             fd_global(contrainte(A,B,L1,L2,R) , state(L1,L2), Susp).
     
     dom_suspensions2([], [], []).
     dom_suspensions2([XA|XAs],[XB|XBs], [dom(XA),dom(XB)|Susp]) :-
             dom_suspensions2(XAs,XBs, Susp).
     
 
% pairwise(I, (L1,L2), (V1,V2)) is true iff L1[I]=V1 AND L2[I]=V2
% Prerequisites : L1,L2 are both lists of FD_var with the same size, I,V1,V2 are FD_var
%                 --- All the FD_vars MUST have a bounded domains ---
% Warning : all the FD_var are supposed to have numeric bounds (inf,sup not managed)
% Maintain bound-consistency on V1,V2 and arc-consistency on I  (To Be Verified)

% Designed for sicstus 4.1.1
% Date : June 2010
% Authors : Arnaud Gotlieb -- INRIA Rennes Bretagne Atlantique
%           Aymeric Hervieu -- INRIA Rennes Bretagne Atlantique
% usage:

% ?- domain([X1,X2,X3,V1,V2],-1,0), pairwise(I, ([X1,X2,X3],[-1,0,-1]), (V1,V2)), I = 3.
%     V1 = X3, V2 = -1.
% ?- pairwise(I, ([X1,X2,X3],[0,-1,-1]), (V1,V2)), V1 = 0, X1 #< 0.
%     I in 2..3, %%  could deduce also V2 = -1 but at the cost of an awakening - suspended ctr 
% ?- pairwise(I, ([X1,X2,X3], [Y1,Y2,Y3]), (V1,V2)), X1=0, X2=0,X3=0.
%     V1 = 0.
% ?- pairwise(I, ([0,-1,-1], [Y1,Y2,Y3]), (0,-1)).
%     I = 1, Y1 = -1.
% ?- pairwise(I, ([0,-1,-1], [Y1,Y2,Y3]), (0,-1)), Y1=0.
%    no
% ?- domain([X1,X2,X3,V1,V2],0,2), pairwise(I, ([X1,X2,X3],[0,0,0]), (V1,V2)), X1 #< 2, X2 #< 2, V1=2.
%    X1 in 0..1,
%    X2 in 0..1,
%    X3 = 2,
%    V2 = 0


 
% pairwise(I, (L1,L2), (V1,V2)) is true iff L1[I]=V1 AND L2[I]=V2
% Prerequisites : L1,L2 are both lists of FD_var with the same size, I,V1,V2 are FD_var
%                 --- All the FD_vars MUST have a bounded domains ---
% Warning : all the FD_var are supposed to have numeric bounds (inf,sup not managed)
% Maintain bound-consistency on V1,V2 and arc-consistency on I  (To Be Verified)

% Designed for sicstus 4.1.1
% Date : June 2010
% Authors : Arnaud Gotlieb -- INRIA Rennes Bretagne Atlantique
%           Aymeric Hervieu -- INRIA Rennes Bretagne Atlantique
% usage:

% ?- domain([X1,X2,X3,V1,V2],-1,0), pairwise(I, ([X1,X2,X3],[-1,0,-1]), (V1,V2)), I = 3.
%     V1 = X3, V2 = -1.
% ?- pairwise(I, ([X1,X2,X3],[0,-1,-1]), (V1,V2)), V1 = 0, X1 #< 0.
%     I in 2..3, %%  could deduce also V2 = -1 but at the cost of an awakening - suspended ctr 
% ?- pairwise(I, ([X1,X2,X3], [Y1,Y2,Y3]), (V1,V2)), X1=0, X2=0,X3=0.
%     V1 = 0.
% ?- pairwise(I, ([0,-1,-1], [Y1,Y2,Y3]), (0,-1)).
%     I = 1, Y1 = -1.
% ?- pairwise(I, ([0,-1,-1], [Y1,Y2,Y3]), (0,-1)), Y1=0.
%    no
% ?- domain([X1,X2,X3,V1,V2],0,2), pairwise(I, ([X1,X2,X3],[0,0,0]), (V1,V2)), X1 #< 2, X2 #< 2, V1=2.
%    X1 in 0..1,
%    X2 in 0..1,
%    X3 = 2,
%    V2 = 0


:- ensure_loaded(library(clpfd)).
:- ensure_loaded(library(lists)).





pairwise(I, (L1,L2), (V1,V2)) :-
        lists:is_list(L1), lists:is_list(L2),
        minmax_suspensions(L1, Susp1, 0, Len1),
        minmax_suspensions(L2, Susp2, 0, Len2),
        Len1 == Len2,
        I in 1 .. Len1,
        !,
        lists:append(Susp1, Susp2, Susp3),
        fd_global(pw(I,L1,L2,V1,V2), state, [dom(I),minmax(V1),minmax(V2)|Susp3]).
%pairwise(_, _, _) :-
%        write('Pairwise constraint problem').

/*
clpfd:dispatch_global(pw(I,L1,L2,V1,V2), state, state, Actions) :-
        pw_solver(I,L1,L2,V1,V2,Actions).
*/
pw_solver(I, L1, L2, V1, V2, Actions) :-
        number(I),
        !,
        lists:nth1(I,L1,X1),
        lists:nth1(I,L2,X2),
        Actions = [exit, call(V1=X1), call(V2=X2)].
pw_solver(I, L1, L2, V1, V2, Actions) :-
%        nl,nl,
        fd_set(I, SetI), fd_set(V1, SetV1), fd_set(V2, SetV2),
        fdset_to_list(SetI, ListI),
        pw_rec(ListI, SetI, L1, L2, SetV1, SetV2, NSetI, [], L1Union, [], L2Union),
        fdset_intersection(SetV1, L1Union, NSetV1),
%        write(L1Union),nl,
        fdset_intersection(SetV2, L2Union, NSetV2),
%        write(L2Union),nl,
        build_actions(NSetI, NSetV1, NSetV2, I,L1, L2, V1,V2,Actions).

pw_rec([], SetI, L1, L2, SetV1, SetV2, SetI, L1U, L1U, L2U, L2U):- !.
pw_rec([Val|S], SetI, L1, L2, SetV1, SetV2, NSetI, L1U, NL1U, L2U, NL2U) :-
        lists:nth1(Val, L1, X1),
        lists:nth1(Val, L2, X2),
        fd_set(X1, SetX1),
        fd_set(X2, SetX2),
        test_disjoint(SetX1, SetV1, Val, SetI, SetI1, L1U, L1Ua),
        test_disjoint(SetX2, SetV2, Val, SetI1, SetI2, L2U, L2Ua),
        pw_rec(S, SetI2, L1, L2, SetV1, SetV2, NSetI, L1Ua, NL1U, L2Ua, NL2U).

test_disjoint(SetX, SetV, Val, SetI, NSetI, LU, LU) :-
        fdset_disjoint(SetX, SetV),
        !,
        fdset_del_element(SetI, Val, NSetI).
test_disjoint(SetX, SetV, Val, SetI, SetI, LU, NLU) :-
        fdset_union(LU, SetX, NLU).

build_actions(NSetI, NSetV1, NSetV2, I,L1,L2,V1,V2,Actions):-
        ( empty_fdset(NSetI) ; empty_fdset(NSetV1) ; empty_fdset(NSetV2)), 
        !,
        Actions = [fail].
build_actions(NSetI, NSetV1, NSetV2, I,L1, L2, V1,V2,Actions):-
        fdset_singleton(NSetI, Ival),
        !,
        lists:nth1(Ival, L1, X1),
        lists:nth1(Ival, L2, X2),
        Actions = [exit, call(I=Ival), call(V1=X1), call(V2=X2)].


%build_actions(NSetI, NSetV1, NSetV2, I,L1, L2, V1,V2,Actions):-
%        ( fdset_singleton(NSetV1, V1val) -> (L1 = [V1=V1val],X=true) ; L1 = [] ),
%        ( fdset_singleton(NSetV2, V2val) -> (L2 = [V2=V2val],X=true) ; L2 = [] ),
%        ( X == true -> lists:append(L1,L2,Actions) ; fail ),
%        !.
build_actions(NSetI, NSetV1, NSetV2, I,L1, L2, V1,V2,Actions):-
        Actions = [I in_set NSetI, V1 in_set NSetV1, V2 in_set NSetV2].
        
%=====================================UTILS============================================
minmax_suspensions([], [], Len, Len).
minmax_suspensions([X|L], [minmax(X)|SL], Len, Len1) :-
        LenI is Len+1,
        minmax_suspensions(L,SL, LenI,Len1).








/*
pairwisegenerator.pro
*/

put_Attr_rec([]).
put_Attr_rec([A|R]):-number(A),!,put_Attr_rec(R).
put_Attr_rec([A|R]):- put_atts(A ,ftAttr(_)),put_Attr_rec(R).

/*
sort(L1,L2)
predicat de tri 
*/

customborneRec([A|R],Min,Max):-    
        get_atts(A,ftAttr(_,I)),!,
        I in Min..Max,
        customborneRec(R,Min,Max).



customborneRec([],_,_).
/*
Borne entre 0 et 1 l'attribut associé à l'état de la feature
*/
borneRec([A|R]):-    
  
       A in 0..1,
        borneRec(R).
borneRec([]).

/*
getLabelVar(+LST_ATTR_VAR,-LST_ATTR).
        LST_ATTR_VAR : list of attributed var : lstAttr([...],Ai)
        LST_ATTR : list of the Ai
*/

getLabelVar([],[]).
getLabelVar([A|R],[Ai|Ri]):-get_atts(A,lstAttr(_,Ai)), getLabelVar(R,Ri).



/*


*/

detectInvalidPairRank(L1,R):-
        sort(L1,L2),
        writeln(L2),
        invalidPRank(L2,R).


  invalidPRank(L2,I):-invalidPRank2(L2,I) .


invalidPRank2([A,B|R],I):-
        A0 is A + 1, B =< A0, !,invalidPRank2([B|R],I).

invalidPRank2([A,B|R],A).

invalidPRank2([A],A).
/*
Fonction de tri :
                
   
*/
my_sort(L1,L2):-writeln('crit_dpthOp'),samsort(crit_dpthOp,L1,L2).

crit_dpthOp(A,B):-
        
         get_atts(A,lstAttr([OpA,Dpth_opA,DpthA],_)),
         get_atts(B,lstAttr([OpB,Dpth_opB,DpthB],_)),
         
         Dpth_opA > Dpth_opB.

crit_dpthFt(A,B):-
        
         get_atts(A,lstAttr([OpA,Dpth_opA,DpthA],_)),
         get_atts(B,lstAttr([OpB,Dpth_opB,DpthB],_)),
         DpthA > DpthB.

crit_op(A,B):-
 
         get_atts(A,lstAttr([OpA,Dpth_opA,DpthA],_)),
         get_atts(B,lstAttr([OpB,Dpth_opB,DpthB],_)),
        operatorComp(OpA,OpB).


/*



crit_dpthOp(A,B):-
        
         get_atts(A,lstAttr([OpA,Dpth_opA,DpthA],_)),
         get_atts(B,lstAttr([OpB,Dpth_opB,DpthB],_)),       
         Dpth_opA > Dpth_opB,!.

crit_dpthOp(A,B):-crit_dpthFt(A,B).

crit_dpthFt(A,B):-
        
         get_atts(A,lstAttr([OpA,Dpth_opA,DpthA],_)),
         get_atts(B,lstAttr([OpB,Dpth_opB,DpthB],_)),
         DpthA > DpthB,!.

crit_dpthFt(A,B):-crit_op(A,B).

crit_op(A,B):-
 
         get_atts(A,lstAttr([OpA,Dpth_opA,DpthA],_)),
         get_atts(B,lstAttr([OpB,Dpth_opB,DpthB],_)),
        operatorComp(OpA,OpB).

*/
operatorComp(xor,and).
operatorComp(xor,opt).
operatorComp(xor,or).
operatorComp(xor,null).

operatorComp(and,opt).
operatorComp(and,or).
operatorComp(and,null).

operatorComp(or,opt).
operatorComp(or,null).
        
operatorComp(opt,null).

/*
pairwiseGenerator(LstFeature,Matrix,PWCTRLST,RANKLST).

A in -1..0, B in -1.. 0 , C in -1..0, LstFeature = [A,B,C],CA = [A1,A2,A3], CB = [B1,B2,B3], CC =[C1,C2,C3] , Matrix = [CA,CB,CC], 
pairwiseGenerator(LstFeature,Matrix,PWCTRLST,RANKLST), 
[pairwise(-1,-1,CA,CB,_A),pairwise(-1,0,CA,CB,_B),pairwise(0,-1,CA,CB,_C),pairwise(0,0,CA,CB,_D),
pairwise(-1,-1,CA,CC,_E),pairwise(-1,0,CA,CC,_F),pairwise(0,-1,CA,CC,_G),pairwise(0,0,CA,CC,_H),
pairwise(-1,-1,CB,CC,_I),pairwise(-1,0,CB,CC,_J),pairwise(0,-1,CB,CC,_K),pairwise(0,0,CB,CC,_L)]
RANKLST = [[_A,_B,_C,_D],[_E,_F,_G,_H],[_I_,_J,_K,_L]].

A in -1..0, B in -1.. 0 , C in -1..0,mutex(A,B),mutex(B,C), LstFeature = [A,B,C],CA = [A1,A2,A3], CB = [B1,B2,B3], CC =[C1,C2,C3] , Matrix = [CA,CB,CC], 
pairwiseGenerator(LstFeature,Matrix,PWCTRLST,RANKLST)

*/


/*
pairwiseGenerator(+LST_FT,+MATRIX,-PWCTRLST,-HLV_LST,-ATTR_HLV_PRED_LST).
        LST_FT : Features list
        MATRIX : data structure
        PWCTRLST : List of pairwise constraints
        HLV_LST : High level Variable list                                    
        ATTR_HLV_PRED_LST : predictate list to set attributes on each HLV
*/

pairwiseGenerator([_|[]],_,[],[],_).
pairwiseGenerator([A|R],[CA|CR],PWCTRLST,HLV_LST,L) :- number(A), !,pairwiseGenerator(R,CR,PWCTRLST,HLV_LST,L).
pairwiseGenerator([A|R],[CA|CR],PWCTRLST,HLV_LST,L) :-   
     R\= [],  pairwiseGenerator2(A,R,CA,CR,PWCTRLST1,HLV_LST1,L),
        pairwiseGenerator(R,CR,PWCTRLST2,HLV_LST2,L), 
        append(PWCTRLST1,PWCTRLST2,PWCTRLST) , 
        append(HLV_LST1,HLV_LST2,HLV_LST).



        
pwunit:-
        Ft = [A,B,C],
         assertRec([a(_,_,[null,0,1])]),  
        put_Attr_rec(Ft) ,
        domain(Ft,0,1),
        matrice(M,3,8),
        pairwiseGenerator(Ft,M,PW,LL,L),
        writeln(PW),
        writeln(LL),writeln(L).

        /*x
        
        
| ?- A in 0..1, B in 0..1 , C in 0..1, D in 0..1,  LstFeature = [A,B,C,D],CA = [A1,A2,A3], CB = [B1,B2,B3], CC =[C1,C2,C3], CD = [D1,D2,D3] ,CtrList = [], Matrix = [CA,CB,CC,DD], pairwiseGenerator(LstFeature,Matrix,PWCTRLST,RANKLST,CtrList)
CA = [A1,A2,A3],
CB = [B1,B2,B3],
CC = [C1,C2,C3],
Matrix = [[A1,A2,A3],[B1,B2,B3],[C1,C2,C3]],
RANKLST = [[_A,_B,_C,_D],[_E,_F,_G,_H],[_I,_J,_K,_L]],
PWCTRLST = [pairwise(-1,-1,[A1,A2,A3],[B1,B2,B3],_A),pairwise(-1,0,[A1,A2,A3],[B1,B2,B3],_B),pairwise(0,-1,[A1,A2,A3],[B1,B2,B3],_C),pairwise(0,0,[A1,A2,A3],[B1,B2,B3],_D),pairwise(-1,-1,[A1,A2,A3],[C1,C2,C3],_E),pairwise(-1,0,[A1,A2,A3],[C1,C2,C3],_F),pairwise(0,-1,[A1,A2,A3],[C1,C2,C3],_G),pairwise(0,0,[A1,A2|...],[C1,C2|...],_H),pairwise(-1,-1,[B1|...],[C1|...],_I),pairwise(...)|...],
LstFeature = [A,B,C],
A in-1..0,
B in-1..0,
C in-1..0 ? ;
no
% source_info
            */


pairwiseGenerator2(_,[],_,[],[],[],L).




pairwiseGenerator2(A,[B|R],CA,[CB|CR],PWCTRLST,HLV_LST,L) :-
        
     
      pairwiseConstraintsGenerator(A,B,CA,CB,PWCTRLST1,HLV_LST1,L),
      pairwiseGenerator2(A,R,CA,CR,PWCTRLST2,HLV_LST2,L),
        append(PWCTRLST1,PWCTRLST2,PWCTRLST) ,
        HLV_LST = [HLV_LST1|HLV_LST2] .



/*

Starting point : Takes Two finite domains variables and generate corresponding pw constraints
pairwsiseConstraintsGenerator(+A,+B,+CA,+CB,-PWCTRLST,-RANKLST)
      A : finite domain variable (in -1 .. 0)
      B : finite domain variable (in -1 .. 0)
      CA : A matrix Column
      CB : B matrix Column
      PWCTRLST : list of pairwiseconstraints
      RANKLST : list of matrix ranks      
      
      
A in -1..0, B in -1..0, CA = [A1,A2,A3], CB = [B1,B2,B3],pairwiseConstraintsGenerator(A,B,CA,CB,PWCTRLST,RANKLST),
PWCTRLST = [pairwise(-1,-1,CA,CB,I1),pairwise(-1,0,CA,CB,I2),pairwise(0,-1,CA,CB,I3),pairwise(0,0,CA,CB,I4)],
RANKLST = [I1,I2,I3,I4]. 
Order doesn't matter           
,a(A,B,Liste),writeln(Liste),
                puttattReq(RANKLST,Liste),printlst(RANKLST),     pairvalidator(A,B,PWCTRLST,PWCTRLST2)      ,pairvalidator(StA,StB,PWCTRLST,PWCTRLST2)
pairwiseConstraintsGenerator(A,B,CA,CB,PWCTRLST2,HLV_LST2,L) :-
      
        
             ((number(A); number(B)) -> Attr = [null,0,0] ) ;
                   ( get_atts(A,ftAttr(NameA)),    get_atts(B,ftAttr(NameB)), a(NameA,NameB,Attr) ),
   
             domainToList(A,LA),
    
   
           avalueGenerator(A,LA,B,CA,CB,PWCTRLST,HLV_LST,Attr),

   pairvalidator(A,B,PWCTRLST,PWCTRLST2,HLV_LST,HLV_LST2,L).

*/

getatt_per(A,B,R):- 
        
        ((number(A) ; number(B)) -> R = [null,0,0] ) ; get_atts(A,ftAttr(NameA)),    get_atts(B,ftAttr(NameB)), a(NameA,NameB,R).

pairwiseConstraintsGenerator(A,B,CA,CB,PWCTRLST2,HLV_LST2,L) :-   
        getatt_per(A,B,Attr),
             domainToList(A,LA),
            avalueGenerator(A,LA,B,CA,CB,PWCTRLST,HLV_LST, Attr ),
 pairvalidator(A,B,PWCTRLST,PWCTRLST2,HLV_LST,HLV_LST2,L).
        
puttattReq([I|Ir],Attr):-
       write(' I0 : '),
       writeln(I),
       put_atts(I,lstAttr(Attr)),
       write(' I1 : ' ),
       writeln(I),
       puttattReq(Ir,Attr).

puttattReq([],_).
        
        


/*
Test
                pairvalidator(A,B,PWCTRLST,PWCTRLST2).
   
   | ?- A in -1..0, B in -1..0, CA = [A1,A2,A3], CB = [B1,B2,B3],pairwiseConstraintsGenerator(A,B,CA,CB,PWCTRLST,RANKLST).
CA = [A1,A2,A3],
CB = [B1,B2,B3],
RANKLST = [_A,_B,_C,_D],
PWCTRLST = [pairwise(-1,-1,[A1,A2,A3],[B1,B2,B3],_A),pairwise(-1,0,[A1,A2,A3],[B1,B2,B3],_B),pairwise(0,-1,[A1,A2,A3],[B1,B2,B3],_C),pairwise(0,0,[A1,A2,A3],[B1,B2,B3],_D)],
A in-1..0,
B in-1..0 ? ;
no
% source_info
            
Constraint handling :
                    % source_info
| ?- A in -1..0, B in -1..0,mutex(A,B), CA = [A1,A2,A3], CB = [B1,B2,B3],pairwiseConstraintsGenerator(A,B,CA,CB,PWCTRLST,RANKLST).
CA = [A1,A2,A3],
CB = [B1,B2,B3],
RANKLST = [_A,_B,_C],
PWCTRLST = [pairwise(-1,-1,[_D,_E,_F],[_G,_H,_I],_A),pairwise(-1,0,[_D,_E,_F],[_G,_H,_I],_B),pairwise(0,-1,[_J,_K,_L],[_M,_N,_O],_C)],
A in-1..0,
B in-1..0 ? ;
no
% source_info
            OK
*/

/*
pairvalidator(FTA,FTB,LstPWCtr)
retire les paires interdites (au cas ou)

*/
pairvalidator(_,_,[],[],[],[],L).

pairvalidator(FTA,FTB,[A|R],[R1|R2],[HLVLVA|HLVLVAR],HLVLVA2RR,L):-

        pairvalidator2(FTA,FTB,A,R1,HLVLVA,HLVLVA2,L),
        
        pairvalidator(FTA,FTB,R,R2,HLVLVAR,HLVLVA2R,L),
              
        append(HLVLVA2,HLVLVA2R,HLVLVA2RR).

pairvalidator2(FTA,FTB, pairwise(I,(CA,CB),(VA,VB)),[],[],[],Ctr) :-
              \+( callRec([FTA=VA, FTB=VB,callRec(Ctr)]) ),
              !.

pairvalidator2(FTA,FTB,Ctr,Ctr,HLVLVA,[HLVLVA],_) :-
             Ctr = pairwise(I,(CA,CB),(VA,VB)).



/*
pairvalidator2(FTA,FTB,Ctr,Ctr,HLVLVA,[HLVLVA],FT):-    Ctr=pairwise(I,(CA,CB),(VA,VB)),(\+(\+(callRec([FTA=VA, FTB=VB,labeling([ff],FT)])))),!.
pairvalidator2(FTA,FTB,pairwise(I,(CA,CB),(VA,VB)),[],_,[],FT).
*/

/*
pairwise(_15783,([_11297,_11330,_11363,_11396,_11429],[_11473,_11506,_11539,_11572,_11605]),(1,1)),
pairvalidator2Test(FTA,FTB,VA,VB):-(\+(\+(callRec([FTA#=VA, FTB#=VB])))),writeln('Cas1').
pairvalidator2Test(FTA,FTB,VA,VB):-(\+(\+(\+(callRec([FTA#=VA, FTB#=VB]))))),writeln('Cas2').
/*
pairvalidator2(FTA,FTB,pairwise(I,(CA,CB),(VA,VB)),[],_,[],L):-writeln('indecis').
*/
/*pairvalidator2(FTA,FTB,pairwise(I,(CA,CB),(VA,VB)),[],_,[],L):- \+(\+(\+(callRec([ FTA#=VA, FTB#=VB,labeling([ff],L)])))),!.
pairvalidator2(FTA,FTB,Ctr,Ctr,HLVLVA,[HLVLVA],L).test0
*/





/*
domainToList(+A,-L).
A in -1..0, domainToList(A,L),L =[-1,0].
No backtrack        
*/

domainToList(A,L) :-
          fd_dom(A,RA),range_to_fdset(RA,SA),fdset_to_list(SA,L).

/*
| ?- A in -1..0, domainToList(A,L),L =[-1,0].
L = [-1,0],
A in-1..0 ? 
*/



/*
avalueGenerator(LA,B,CA,CB,PWCTRLST,RANKLST).
LA : int List
      B : finite domain variable (in -1 .. 0)
      CA : A matrix Column
      CB : B matrix Column
      PWCTRLST : list of pairwiseconstraints
      RANKLST : list of matrix ranks    
      
      A = [-1,0], B in -1..0, CA = [A1,A2,A3], CB = [B1,B2,B3],avalueGenerator(A,B,CA,CB,PWCTRLST,RANKLST),
PWCTRLST = [pairwise(-1,-1,CA,CB,I1),pairwise(-1,0,CA,CB,I2),pairwise(0,-1,CA,CB,I3),pairwise(0,0,CA,CB,I4)],
RANKLST = [I1,I2,I3,I4]. 

      \+(\+(callRec([write('A : '),writeln(A),write('VA : '),writeln(VA),write('B : '),writeln(B),A=VA,pairListGenerator(VA,B,PairList),write('VB : '),writeln(B),writeln(PairList),assert(p1(PairList))]))),
   
*/

avalueGenerator(_,[],_,_,_,[],[],_).
avalueGenerator(A,[VA|R],B,CA,CB,PWCTRLST,HLV_LST,Attr) :-
            
        \+(\+(callRec([A=VA,pairListGenerator(VA,B,PairList),assert(p1(PairList))]))),
        p1(PairList),pairsGeneretor(PairList,CA,CB,PWCTRLST1,HLV_LST1,Attr),retract(p1(_)),
        avalueGenerator(A,R,B,CA,CB,PWCTRLST2,HLV_LST2,Attr),
        append(PWCTRLST1,PWCTRLST2,PWCTRLST),
        append(HLV_LST1,HLV_LST2,HLV_LST).
/*
avalueGenerator(A,[VA|R],B,CA,CB,PWCTRLST,HLV_LST,Attr) :-writeln(A), writeln('pb'),avalueGenerator(A,R,B,CA,CB,PWCTRLST,HLV_LST,Attr).
*/
/* Test : 

| ?-  A in -1..0 ,LA = [-1,0], B in -1..0, CA = [A1,A2,A3], CB = [B1,B2,B3],avalueGenerator(A,LA,B,CA,CB,PWCTRLST,RANKLST).
CA = [A1,A2,A3],
CB = [B1,B2,B3],
LA = [-1,0],
RANKLST = [_A,_B,_C,_D],
PWCTRLST = [pairwise(-1,-1,[_E,_F,_G],[_H,_I,_J],_A),pairwise(-1,0,[_E,_F,_G],[_H,_I,_J],_B),pairwise(0,-1,[_K,_L,_M],[_N,_O,_P],_C),pairwise(0,0,[_K,_L,_M],[_N,_O,_P],_D)],
A in-1..0,
B in-1..0 ? ;
no
% source_info
 */

/*
pairListGenerator(VA,B,PairList) 
Va : int
 B : FiniteDomaine Variable
 PairList : list of pairs between VA and B
VA = 1, B in -1..0, pairListGenerator(VA,B,PairList) ,PairList = [[1,-1],[1,0]].

pairwiseGenerator2(A,[B|R],CA,[CB|CR],PWCTRLST,HLV_LST,L) :-number(B),!,
            
        pairwiseGenerator2(A,R,CA,CR,PWCTRLST,HLV_LST,L).
*/

pairListGenerator(VA,B,PairList) :-   number(B),!,
                                    
                                  pairListUnitGenerator(VA,[B],PairList).
pairListGenerator(VA,B,PairList) :-   
                                        domainToList(B,LB),
                                  pairListUnitGenerator(VA,LB,PairList).

/*
| ?- VA = 1, B in -1..0, pairListGenerator(VA,B,PairList).
VA = 1,
PairList = [[1,-1],[1,0]],
B in-1..0 ? ;
no
% source_info
*/

/*
pairListUnitGenerator(VA,LB,PairList) 
Va : int
 LB : int List
 PairList : list of pairs between VA and B
VA = 1, LB = [-1,0], pairListUnitGenerator(VA,LB,PairList) ,PairList = [[-1,-1],[-1,0]].
*/

 pairListUnitGenerator(_,[],[]).
 pairListUnitGenerator(VA,[B|R],[[VA,B]|Res]):-
                      pairListUnitGenerator(VA,R,Res).                              

/*
  
| ?- VA = 1, LB = [-1,0], pairListUnitGenerator(VA,LB,PairList).
LB = [-1,0],
VA = 1,
PairList = [[1,-1],[1,0]] ? ;
no
% source_info
| ?- */


/*
 pairsGeneretor(PairList,CA,CB,PWCTRLST,RANKLST).
        PairList : [[-1,-1],[1,1]...]
      CA : A matrix Column
      CB : B matrix Column
      PWCTRLST : list of pairwiseconstraints
      RANKLST : list of matrix ranks              

PairList = [[1,1],[0,0]], CA = [A1,A2,A3], CB = [B1,B2,B3],pairsGeneretor(PairList,CA,CB,PWCTRLST,RANKLST),
PWCTRLST = [pairwise(0,0,CA,CB,I1),pairwise(1,1,CA,CB,I2)],
RANKLST = [I1,I2,I3,I4]. 
Order doesn't matter        
No backtrack         

*/

 pairsGeneretor([],_,_,[],[],_).
 pairsGeneretor([[VA,VB]|R],CA,CB,PWCTRLST,HLV_LST,Attr) :-put_atts(HLVA,lstAttr(Attr,I)),
                                                        PWCTRLST1 = [pairwise(I,(CA,CB),(VA,VB))],
                                                        HLV_LST1 = [HLVA],
                                                        pairsGeneretor(R,CA,CB,PWCTRLST2,HLV_LST2,Attr),
                                                        append(PWCTRLST1,PWCTRLST2,PWCTRLST),
                                                        append(HLV_LST1,HLV_LST2,HLV_LST).
/*
| ?- PairList = [[1,1],[0,0]], CA = [A1,A2,A3], CB = [B1,B2,B3],pairsGeneretor(PairList,CA,CB,PWCTRLST,RANKLST).
CA = [A1,A2,A3],
CB = [B1,B2,B3],
RANKLST = [_A,_B],
PairList = [[1,1],[0,0]],
PWCTRLST = [pairwise(1,1,[A1,A2,A3],[B1,B2,B3],_A),pairwise(0,0,[A1,A2,A3],[B1,B2,B3],_B)] ? ;
no
% source_info
| ?- 
*/






contrainteFD2(M,LF,I) :-I > 0, ligne(M,I,L),
                        validLine(LF,L),I2 is I -1,
                        contrainteFD1(M,LF,I2).
contrainteFD2(_,_,0).

/*

LF = [A,B,C],domain(LF,-1,0),mutex(A,B),Values = [0,0,0],  validLine(LF,Values)
*/

validLine(LF,Valeur) :-lineConstraint(Valeur,_,LF).
  
testCtrFDV2:-
        Features = [A,B,C], 
        domain(Features,0,1),
    matrice(M,3,5),
        limitMatrix(M,Features),
        Ctr = [and(A,[B,C])],
        contrainteFDV2(M,Features,Ctr,R).


contrainteFDV2(M,LF,CTR,Res):-nbligne(M,I),
                              contrainteFD2V2(M,LF,I,CTR,Res).
        
        
    

contrainteFD2V2(M,LF,I,CTR,R2) :-I > 0,
                                 ligne(M,I,L),                        
                                 copy_term((CTR, LF),  (Res, L)),             
                                 I2 is I -1,
                                 contrainteFD2V2(M,LF,I2,CTR,R),
                                 append(Res,R,R2).

contrainteFD2V2(_,_,0,_,[]).


/*
Nb : number of unconstrainted lines
*/



contrainteFDV2(M,LF,CTR,Res,Nb):- 
                                nbligne(M,I),
                              contrainteFD2V2(M,LF,I,CTR,Res,Nb).
        
        

contrainteFD2V2(M,LF,I,CTR,R2,Nb) :-Nb > 0,
                                    Nb2 is Nb - 1 ,
                                 I2 is I -1,
                                 contrainteFD2V2(M,LF,I2,CTR,R2,Nb2) .


contrainteFD2V2(M,LF,I,CTR,R2,Nb) :-Nb = 0,
                                 ligne(M,I,L),                        
                                 copy_term((CTR, LF),  (Res, L)),             
                                 I2 is I -1,
                                 contrainteFD2V2(M,LF,I2,CTR,R,Nb),
                                 append(Res,R,R2).

contrainteFD2V2(_,_,0,_,[],0).





putAttsRec([A|R],[B|Rb]):-
                put_atts(A,ftAttr(_,B)),
                putAttsRec(R,Rb).
putAttsRec([],[]).



/* variable(M,V) : obtention de la liste des variable de M */
/* variable(M,V) : obtention de la liste des variable de M */

nvariable2(_,[],0).
nvariable2([A|R],L,I):-I \=0,
                       I2 is I -1,
                       nvariable2(R,L2,I2),
                       append(A,L2,L).

nvariable(M,L,I):-mytranspose(M,M2),
                  nvariable2(M2,L,I).


submatrix(M,M2,I):-mytranspose(M,MT),
                   submatrix2(MT,M2,I).

submatrix2([A|R],[A|R2],I):- I\=0,
                             I2 is I -1, 
                             submatrix2(R,R2,I2).
submatrix2(_,[],0).




        


/*
ToolS.pro
*/


writeStatV2(Nbft,Temps,NbConfig,NBVraiConf, File,Labeling, Sort):-
        open(File, append, Stream),
        write(Stream,Nbft),
        write(Stream,';'),
        write(Stream,Temps),
        write(Stream,';'),
        write(Stream,NbConfig),
        write(Stream,';'),
        write(Stream,NBVraiConf),
        write(Stream,';'),
        write(Stream,Labeling),
        write(Stream,';'),
        write(Stream,Sort),
        nl(Stream),
        close(Stream).


writeln(A) :- write(A), write('\n').
writeStat(Nbft,Temps,NbConfig,NBVraiConf, File):-open(File, append, Stream),write(Stream,Nbft),write(Stream,';'),write(Stream,Temps),write(Stream,';'),write(Stream,NbConfig),write(Stream,';'),write(Stream,NBVraiConf),write(Stream,';'),nl(Stream),close(Stream).

writeStat0(Etiquette,File):-open(File, append, Stream),write(Stream,Etiquette),write(Stream,';'),close(Stream).


writeStat2(Etiquette,Nbft,Temps,NbConfig,ValeurdeK, File):-open(File, append, Stream),write(Stream,Nbft),write(Stream,';'),write(Stream,Temps),write(Stream,';'),write(Stream,NbConfig),write(Stream,';'),write(Stream,ValeurdeK),write(Stream,';'),nl(Stream),close(Stream).
writeMat(File,M) :- mytranspose(M,M2), open(File, append, Stream),writeMatrice(M2,Stream),nl(Stream),close(Stream).


writeMat2(File,M2) :- open(File, append, Stream),nl(Stream),writeMatrice(M2,Stream),nl(Stream),close(Stream).


writeMatrice([A|B],S) :- writeligne(A,S), write(S,'\n'), writeMatrice(B,S).
writeMatrice([],_).

writeligne([A|[]],S):- write(S,A),!.
writeligne([A|B],S):- write(S,A), write(S,','), writeligne(B,S).
writeligne([],_).

writeMatrixScr(M1):-mytranspose(M1,M2),writeMatrixScr0(M2).
writeMatrixScr0([A|B]) :- writelineScr(A), write('\n'), writeMatrixScr0(B).
writeMatrixScr0([]).

writelineScr([A|B]):- write(A), write(','), writelineScr(B).
writelineScr([]).


writeMatVal(File,M) :- open(File, write, Stream),write(Stream,M),write(Stream,'.'),nl(Stream),close(Stream).


writedom([A|R]) :- fd_set(A,J),writeln(J),writedom(R).
writedom([]). 

/*
Ouverture d'un fichier résultat et chargement de la matrice en memoire.
attention il faut la transposer pour revenir à la strucuture de donnée de pariwise
*/


readM(File,M) :- open(File, read, Stream),readFile(M,Stream),close(Stream).

readFile(M,Stream):- read(Stream,M).
        
/* -*- Tableau -*- */

/*
Transpose(M1,M2)
*/

mytranspose(M1,M2):- nbligne(M1,_), creeMat(M1,M2,1),!.
creeMat(M1,[],K) :- nbligne(M1,I), K is I +1.
creeMat(M1,R,I):-nbligne(M1,K), I =< K, ligne(M1,I,Li), I2 is I +1, R = [Li|M12], creeMat(M1,M12,I2).

/* matrice(M,X,Y) : Matrice M, X : nombre de colonnes, et y nombre de lignes */
/* Crée une liste de colonne */
matrice([],0,_).
matrice([L|M],X,Y) :- X >0 , X1 is X -1 ,length(L,Y), matrice(M, X1, Y) .  



/* elementdeL(L,I,E) :  L : liste, I : rang de l'élément, E : élement */
/*
nth(?N, ?List, ?Element)
*/
elementdeL([A|_],1,A).
elementdeL([_|B],I,C):- I >1 , I1 is I-1 , elementdeL(B,I1 ,C).

/* Obtenir Colonne*/
/* Colonne(M,I,C) */
/* M : matrice définie précedement, I : l'indice de colonne, C : la colonne résultante */ 

colonne(M,I,C):- elementdeL(M,I,C).


/*
Nbligne
*/


nbligne([A|_],L):- length(A,L).
/* Obtenir Ligne */
/* ligne(M,I,L) : M : matrice définie précédement, I : indice de ligne, L : la ligne resultante */

ligne([],_,[]).

ligne([A|R],I,[B|T]) :-nth1(I,A,B) , ligne(R,I,T).

/* variable(M,V) : obtention de la liste des variable de M */

variable([],[]).
variable([A|R],L):- variable(R,L2),append(A,L2,L). 

/* -*- Fin de la définition de la structure de donnée -*- */

/* Outils */

/*
permet de tester si une variable appartient à une lsite, retire la variable de la liste
*/
appartient(X,[A|N],[A|N2]):- appartient(X,N,N2).
appartient(X,[A|N],N):- A == X.
/*
spit(L1,L2,L3,L4) : 
L1 : liste d'éléments
L2 : liste contenant certain élément de L1,
L3 : liste des éléments de L1 présent dans L2
L4 : liste des éléments de L2 sans les éléments de L1                                           
*/


value([],[]).
value([A|R],[B|R2]) :-
        A = B, value(R,R2).

callRec([]).

callRec([A|T]) :-
   
        call(A),
        callRec(T).



 writeInvalidCtr(File,M):- open(File, append, Stream),write(Stream,M),write(Stream,'.'),nl(Stream),close(Stream).


split([],LR,[],LR).
split([A|N],Lx,L1,L) :- (appartient(A,Lx,Ln) -> L1 = [A|L11],split(N,Ln,L11,L));split(N,Lx,L1,L).

/*
RANKLST = [[_A,_B,_C,_D],[_E,_F,_G,_H],[_I_,_J,_K,_L]],flatten(RANKLST,Lst),Lst =
[_A,_B,_C,_D,_E,_F,_G,_H,_I_,_J,_K,_L]
*/
flatten([L|T],Lst):- flatten(T,Lst2), append(L,Lst2,Lst).
flatten([],[]).


limitMatrix([A|R],[FA|FR]):- fd_min(FA,MinA),fd_max(FA,MaxA), domain(A,MinA,MaxA), limitMatrix(R,FR).
limitMatrix([],[]).     



/* Initialisation du problème */
limit([],_,_).
limit([A|R],Min,Max) :- fdset_interval(A,Min,Max),limit(R,Min,Max).
/*
Compte le nolmbre d'occurence de -1 et de 0, rend la liste des variables non instanciées
       N : nbre de 0
   M : nbre de -1
*/


        occur_cpt([LF|LFs],LF0,N,M) :- 
                LF == 0,!,
                 occur_cpt(LFs,LF0,N1,M), N is N1 +1 .
               
     
        occur_cpt([LF|LFs],LF0,N,M) :-  
                LF == -1,!,
               occur_cpt(LFs,LF0,N,M1), M is M1 +1 .

        occur_cpt([LF|LFs],LF0,N,M):-
                LF0 = [LF|LF02],
                occur_cpt(LFs,LF02,N,M).

       occur_cpt([],[],0,0).



domainRec([A|R],V1,V2):-domain(A,V1,V2),
                        domainRec(R,V1,V2).
domainRec([],_,_).
   

assertRec([A|T]):-assert(A),
                  assertRec(T).
assertRec([]).

                                                                     
                                                                     
                                                                     
                                             
% Global Constraint   max50(V,L) is true, iff V is the maximum of L, provided that any variable in L being >= 50 is withdrawn
% Behaviour similar to the SICStus GC  maximum(V,L) [apart from the 50 limit]
% but less deductive than maximum (see example below)

% Usage in Pacogen to avoid the minimization of the maximum of rank, when it is greater than 50  
% Designed with sicstus 4.2.0
% Date : Apr. 2012
% Authors : Arnaud Gotlieb -- SIMULA RESEARCH LAB.
% usage:
%| ?- I1 in 0..3, I2 in 0..49, I3 in 2..3, I4 in 0..7, max50(V, [I1,I2,I3,I4]).
% I1 in 0..3, I2 in 0..49, I3 in 2..3, I4 in 0..7, V in 2..49 ? 

%| ?- I1 in 0..3, I2 in 50..50, I3 in 2..3, I4 in 0..7, max50(V, [I1,I2,I3,I4]).
%I2 = 50, I1 in 0..3, I3 in 2..3, I4 in 0..7, V in 2..7 ? 

%| ?- I1 in 0..3, I2 in 0..49, I3 in 2..3, I4 in 0..7, max50(V, [I1,I2,I3,I4]), labeling([minimize(V)], [I1,I2,I3,I4]).
%V = 2, I1 = 0, I2 = 0, I3 = 2, I4 = 0 ? 

%| ?- I1 in 0..3, I2 in 50..55, I3 in 2..3, I4 in 0..7, max50(V, [I1,I2,I3,I4]), labeling([minimize(V)], [I1,I2,I3,I4]).
%V = 2,I1 = 0,I2 = 50,I3 = 2,I4 = 0 ? 

%%% max50 less deductive than maximum
% | ?- I1 in 0..4, I2 in 2..3, max50(V, [I1,I2]), V #>3.
% V = 4, I1 in 0..4, I2 in 2..3 ? 

% | ?- I1 in 0..4, I2 in 2..3, maximum(V, [I1,I2]), V #>3.
% V = 4, I1 = 4, I2 in 2..3 ? 


:- use_module(library(clpfd)).
:- use_module(library(lists)).

clpfd:dispatch_global(max50_ctr(V,LI,Val), state, state, Actions) :-
            max50_solver(V, LI,Val, Actions).

max50(V, LI,Val) :-
        is_list(LI),
        LI \== [],
        domain([V|LI],-134217728, 134217727),    % A is 1<<27, Min is -A, Max is A-1, domain([X], Min, Max).
        !,
        minmax(LI, DOMLI),
        fd_global(max50_ctr(V, LI,Val), state, [minmax(V)|DOMLI]).
max50(V, LI,Val) :-
        write('Pbm: type_error').

max50_solver(V, [I],Val, Actions):-
        !,
        Actions = [call(V=I),call(I #< Val),exit].
max50_solver(V, LI,Val, Actions):-
        number(V),
        V < Val,
        !,
        LI = [I|LIs],
        fd_min(I, MAXBMIN),
        fd_max(I, MAXBMAX),
        minmax_list(LIs, V, MAXBMIN, MAXBMAX, NMAXBMIN, NMAXBMAX,Val),
        V >= NMAXBMIN,
        V =< NMAXBMAX,
        Actions = [exit].
max50_solver(V, LI,Val, Actions) :-
         LI = [I|LIs],
         fd_min(I, MAXBMIN),
         fd_max(I, MAXBMAX),
         minmax_list(LIs, V, MAXBMIN, MAXBMAX, NMIN, NMAX,Val),
         Actions = [V in NMIN .. NMAX].


minmax_list([], _V, NMIN, NMAX, NMIN, NMAX,Val).
minmax_list([I|LI], V, MAXBMIN, MAXBMAX, NMIN, NMAX,Val) :-
        fd_min(I, MIN),
        MIN < Val,
        !,
        fd_max(I, MAX),
        ( MIN > MAXBMIN -> NMAXBMIN = MIN ; NMAXBMIN =  MAXBMIN),
        ( MAX > MAXBMAX -> NMAXBMAX = MAX ; NMAXBMAX =  MAXBMAX),
        minmax_list(LI, V, NMAXBMIN, NMAXBMAX, NMIN, NMAX,Val).
minmax_list([I|LI], V, MAXBMIN, MAXBMAX, NMIN, NMAX,Val) :-
        fd_min(I, MIN),
        MIN >= Val,
        minmax_list(LI, V, MAXBMIN, MAXBMAX, NMIN, NMAX,Val).


minmax([],[]).
minmax([I|Is], [minmax(I)|Ls]) :-
      minmax(Is,Ls).

