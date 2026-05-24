#include "pdp_ga.h"
#include "pdp_tabu.h"
#include "pdp_types.h"
#include "pdp_fitness.h"
#include "pdp_cache.h"
#include "pdp_init.h"
#include <algorithm>
#include <random>
#include <map>
#include <set>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>
#include <numeric>
#include <unordered_set>
using namespace std;

// ---- Bridge: Chromosome → vector<int> for evaluateWithCache ----
static vector<int> chrToIds(const Chromosome& c) {
    vector<int> v; v.reserve(c.size());
    for (auto& g : c) v.push_back(g.node_id);
    return v;
}
static PDPSolution evalChr(const Chromosome& c, const PDPData& d, SolutionCache& cache) {
    return evaluateWithCache(chrToIds(c), d, cache);
}

// ---- AdaptiveParams ----
struct AdaptiveParams {
    vector<double> crossoverSuccess, crossoverRates;
    vector<int>    crossoverUsage;
    vector<double> mutationSuccess, mutationRates;
    vector<int>    mutationUsage;
    double currentMutationRate, baseMutationRate;
    int noImprovementCount;

    AdaptiveParams(double base) :
        crossoverSuccess(4,0), crossoverRates(4,0.25), crossoverUsage(4,0),
        mutationSuccess(6,0),  mutationRates(6,1.0/6), mutationUsage(6,0),
        currentMutationRate(base), baseMutationRate(base), noImprovementCount(0) {}

    void updateCrossoverSuccess(int t, bool ok){ crossoverUsage[t]++; if(ok) crossoverSuccess[t]++; }
    void updateMutationSuccess (int t, bool ok){ mutationUsage[t]++;  if(ok) mutationSuccess[t]++;  }

    void adaptCrossoverRates() {
        double tot=0; for(int i=0;i<4;i++) if(crossoverUsage[i]>0) tot+=crossoverSuccess[i]/crossoverUsage[i];
        if(tot>0) for(int i=0;i<4;i++) crossoverRates[i]= crossoverUsage[i]>0? 0.1+0.8*(crossoverSuccess[i]/crossoverUsage[i])/tot : 0.25;
    }
    void adaptMutationRates() {
        double tot=0; for(int i=0;i<6;i++) if(mutationUsage[i]>0) tot+=mutationSuccess[i]/mutationUsage[i];
        if(tot>0) for(int i=0;i<6;i++) mutationRates[i]= mutationUsage[i]>0? 0.05+0.9*(mutationSuccess[i]/mutationUsage[i])/tot : 1.0/6;
    }
    void adaptMutationRate() {
        if(noImprovementCount>5)     currentMutationRate=min(0.5, currentMutationRate*1.2);
        else if(noImprovementCount<2) currentMutationRate=max(baseMutationRate*0.5, currentMutationRate*0.9);
    }
    int selectCrossoverType(mt19937& g){ discrete_distribution<> d(crossoverRates.begin(),crossoverRates.end()); return d(g); }
    int selectMutationType (mt19937& g){ discrete_distribution<> d(mutationRates.begin(), mutationRates.end());  return d(g); }
};

struct OnlineSurrogate {
    int n=0; double sumX=0,sumY=0,sumXX=0,sumXY=0;
    void update(double x,double y){n++;sumX+=x;sumY+=y;sumXX+=x*x;sumXY+=x*y;}
    bool ready() const { return n>=30; }
    double predict(double x, double fb) const {
        if(n<2) return fb;
        double d=n*sumXX-sumX*sumX;
        if(fabs(d)<1e-9) return fb;
        double a=(n*sumXY-sumX*sumY)/d, b=(sumY-a*sumX)/n, y=a*x+b;
        return isfinite(y)?y:fb;
    }
};

// ---- Edge novelty (Chromosome version, compare node_ids) ----
static double edgeNoveltyScore(const Chromosome& seq, const Chromosome& ref) {
    if(seq.size()<2||ref.size()<2||seq.size()!=ref.size()) return 1.0;
    unordered_set<long long> refE; refE.reserve(ref.size()*2);
    auto ek=[](int a,int b){ return (static_cast<long long>(a)<<32)^static_cast<unsigned>(b); };
    for(size_t i=0;i+1<ref.size();i++) refE.insert(ek(ref[i].node_id,ref[i+1].node_id));
    int overlap=0;
    for(size_t i=0;i+1<seq.size();i++) if(refE.count(ek(seq[i].node_id,seq[i+1].node_id))) overlap++;
    return 1.0-(double)overlap/max(1,(int)seq.size()-1);
}

// ---- Perturbation operators ----
Chromosome doubleBridgePerturbation(const Chromosome& seq, mt19937& gen) {
    int n=seq.size(); if(n<8) return seq;
    uniform_int_distribution<> d(1,n-2);
    set<int> cs; vector<int> cuts;
    while((int)cuts.size()<3){ int c=d(gen); if(!cs.count(c)){cuts.push_back(c);cs.insert(c);} }
    sort(cuts.begin(),cuts.end());
    Chromosome r;
    for(int i=0;      i<cuts[0];i++) r.push_back(seq[i]);
    for(int i=cuts[1];i<cuts[2];i++) r.push_back(seq[i]);
    for(int i=cuts[0];i<cuts[1];i++) r.push_back(seq[i]);
    for(int i=cuts[2];i<n;      i++) r.push_back(seq[i]);
    return r;
}

Chromosome ruinRecreatePerturbation(const Chromosome& seq, mt19937& gen, double ratio) {
    int n=seq.size(), k=max(2,(int)(n*ratio));
    vector<int> idx(n); iota(idx.begin(),idx.end(),0);
    shuffle(idx.begin(),idx.end(),gen);
    set<int> rm(idx.begin(),idx.begin()+k);
    vector<Gene> removed, remaining;
    for(int i=0;i<n;i++) (rm.count(i)?removed:remaining).push_back(seq[i]);
    shuffle(removed.begin(),removed.end(),gen);
    for(auto& g:removed){
        uniform_int_distribution<> p(0,(int)remaining.size());
        remaining.insert(remaining.begin()+p(gen),g);
    }
    return remaining;
}

// ---- Crossover helpers ----
// Split: extract customer genes + record special-node positions from parent1
static void splitChromosome(const Chromosome& p,
                             vector<Gene>& custs,
                             vector<pair<int,Gene>>& specials) {
    for(int i=0;i<(int)p.size();i++) {
        if(p[i].node_id>0) custs.push_back(p[i]);
        else                specials.push_back({i,p[i]});
    }
}

// Reassemble: fill child_cust into skeleton, inserting specials at original relative positions
static Chromosome reassemble(const vector<Gene>& child_cust,
                              const vector<pair<int,Gene>>& specials,
                              int parent_len) {
    // Build a result of size parent_len
    // specials are at their original indices in parent1
    Chromosome result(parent_len);
    // Mark special positions
    set<int> spos; for(auto& s:specials) spos.insert(s.first);
    // Fill specials
    for(auto& s:specials) result[s.first]=s.second;
    // Fill customer slots in order
    int ci=0;
    for(int i=0;i<parent_len;i++) {
        if(!spos.count(i)) result[i]=child_cust[ci++];
    }
    return result;
}

// ---- One-Point Crossover (free-style, includes 0/-1) ----
Chromosome onePointCrossover(const Chromosome& p1, const Chromosome& p2, mt19937& gen) {
    int n = (int)p1.size();
    if (n == 0 || (int)p2.size() != n) return p1;

    uniform_int_distribution<> d(1, n - 1);
    int cut = d(gen);

    Chromosome child;
    child.reserve(n);

    set<int> used_customers;
    for (int i = 0; i < cut; ++i) {
        child.push_back(p1[i]);
        if (p1[i].node_id > 0) used_customers.insert(p1[i].node_id);
    }

    for (int i = 0; i < n && (int)child.size() < n; ++i) {
        const Gene& g = p2[i];
        if (g.node_id > 0) {
            if (used_customers.count(g.node_id)) continue;
            used_customers.insert(g.node_id);
        }
        child.push_back(g);
    }

    return child;
}

// ---- OX Crossover (free-style, allows 0/-1) ----
Chromosome orderCrossover(const Chromosome& p1, const Chromosome& p2, mt19937& gen) {
    int n = (int)p1.size();
    if (n == 0 || (int)p2.size() != n) return p1;

    uniform_int_distribution<> d(0, n - 1);
    int a = d(gen), b = d(gen);
    if (a > b) swap(a, b);

    Chromosome child(n, Gene(0));
    for (auto& g : child) g.resupply_vector.clear();

    set<int> used_customers;
    for (int i = a; i <= b; ++i) {
        child[i] = p1[i];
        if (p1[i].node_id > 0) used_customers.insert(p1[i].node_id);
    }

    int ci = (b + 1) % n;
    int pi = (b + 1) % n;
    while (ci != a) {
        const Gene& cand = p2[pi];
        if (cand.node_id > 0) {
            if (!used_customers.count(cand.node_id)) {
                child[ci] = cand;
                used_customers.insert(cand.node_id);
                ci = (ci + 1) % n;
            }
        } else {
            child[ci] = cand;
            ci = (ci + 1) % n;
        }
        pi = (pi + 1) % n;
    }

    return child;
}

// ---- PMX Crossover ----
Chromosome pmxCrossover(const Chromosome& p1, const Chromosome& p2, mt19937& gen) {
    vector<Gene> c1, c2;
    vector<pair<int, Gene>> spec;
    splitChromosome(p1, c1, spec);
    for (auto& g : p2) if (g.node_id > 0) c2.push_back(g);

    int n = (int)c1.size();
    if (n == 0) return p1;

    uniform_int_distribution<> d(0, n - 1);
    int a = d(gen), b = d(gen);
    if (a > b) swap(a, b);

    vector<Gene> child = c1;
    map<int, int> mapping; // c1.node_id -> c2.node_id
    for (int i = a; i <= b; ++i) {
        mapping[c1[i].node_id] = c2[i].node_id;
        child[i] = c2[i];
    }

    map<int, Gene> gene1, gene2;
    for (const auto& g : c1) gene1[g.node_id] = g;
    for (const auto& g : c2) gene2[g.node_id] = g;

    for (int i = 0; i < n; ++i) {
        if (i >= a && i <= b) continue;
        int v = c1[i].node_id;
        while (mapping.count(v)) v = mapping[v];
        auto it2 = gene2.find(v);
        if (it2 != gene2.end()) child[i] = it2->second;
        else if (gene1.count(v)) child[i] = gene1[v];
    }

    return reassemble(child, spec, (int)p1.size());
}

// ---- CX Crossover ----
Chromosome cycleCrossover(const Chromosome& p1, const Chromosome& p2, mt19937& gen) {
    vector<Gene> c1, c2;
    vector<pair<int, Gene>> spec;
    splitChromosome(p1, c1, spec);
    for (auto& g : p2) if (g.node_id > 0) c2.push_back(g);

    int n = (int)c1.size();
    if (n == 0) return p1;

    map<int, int> pos1;
    for (int i = 0; i < n; ++i) pos1[c1[i].node_id] = i;

    vector<Gene> child(n, Gene(0));
    for (auto& g : child) g.resupply_vector.clear();

    vector<bool> visited(n, false);
    bool take_from_p1 = true;
    for (int start = 0; start < n; ++start) {
        if (visited[start]) continue;
        int idx = start;
        do {
            visited[idx] = true;
            child[idx] = take_from_p1 ? c1[idx] : c2[idx];
            int next_id = take_from_p1 ? c2[idx].node_id : c1[idx].node_id;
            idx = pos1.count(next_id) ? pos1[next_id] : start;
        } while (idx != start);
        take_from_p1 = !take_from_p1;
    }

    return reassemble(child, spec, (int)p1.size());
}

// ---- Mutation operators (operate on full Chromosome incl. 0/-1) ----
void swapMutation(Chromosome& s, mt19937& g) {
    if(s.size()<2) return;
    uniform_int_distribution<> d(0,s.size()-1);
    swap(s[d(g)],s[d(g)]);
}
void inversionMutation(Chromosome& s, mt19937& g) {
    if(s.size()<2) return;
    uniform_int_distribution<> d(0,s.size()-1);
    int a=d(g),b=d(g); if(a>b) swap(a,b);
    reverse(s.begin()+a,s.begin()+b+1);
}
void scrambleMutation(Chromosome& s, mt19937& g) {
    if(s.size()<2) return;
    uniform_int_distribution<> d(0,s.size()-1);
    int a=d(g),b=d(g); if(a>b) swap(a,b);
    shuffle(s.begin()+a,s.begin()+b+1,g);
}
void insertionMutation(Chromosome& s, mt19937& g) {
    if(s.size()<2) return;
    uniform_int_distribution<> d(0,s.size()-1);
    int from=d(g),to=d(g); if(from==to) return;
    Gene elem=s[from]; s.erase(s.begin()+from);
    int ins=to; if(to>from) ins--;
    ins=max(0,min(ins,(int)s.size()));
    s.insert(s.begin()+ins,elem);
}
void displacementMutation(Chromosome& s, mt19937& g) {
    if(s.size()<3) return;
    uniform_int_distribution<> d(0,s.size()-1);
    int a=d(g),b=d(g); if(a>b) swap(a,b);
    if(a==b){ if(b<(int)s.size()-1) b++; else if(a>0) a--; else return; }
    Chromosome seg(s.begin()+a,s.begin()+b+1);
    s.erase(s.begin()+a,s.begin()+b+1);
    vector<int> valid;
    for(int i=0;i<=(int)s.size();i++) valid.push_back(i);
    if(valid.empty()) return;
    uniform_int_distribution<> pd(0,(int)valid.size()-1);
    int ins=valid[pd(g)];
    s.insert(s.begin()+ins,seg.begin(),seg.end());
}

// ---- Drone Resupply Mutation (NEW) ----
void droneResupplyMutation(Chromosome& seq, const PDPData& data, mt19937& gen) {
    // Collect indices with non-empty resupply_vector
    vector<int> sources;
    for(int i=0;i<(int)seq.size();i++)
        if(seq[i].node_id>0 && !seq[i].resupply_vector.empty())
            sources.push_back(i);
    if(sources.empty()) return;
    uniform_int_distribution<> sd(0,(int)sources.size()-1);
    int srcIdx=sources[sd(gen)];
    auto& rv=seq[srcIdx].resupply_vector;
    // Pop random package
    uniform_int_distribution<> pd(0,(int)rv.size()-1);
    int pick=pd(gen);
    int pkg=rv[pick];
    rv.erase(rv.begin()+pick);
    // Find the delivery node for this package (pkg itself is the customer node)
    // Insert into resupply_vector of a random Gene B (node_id>0) before pkg's position
    int pkg_pos=-1;
    for(int i=0;i<(int)seq.size();i++) if(seq[i].node_id==pkg){pkg_pos=i;break;}
    // Candidates: node_id>0, index < pkg_pos (or any if pkg not found)
    vector<int> cands;
    for(int i=0;i<(int)seq.size();i++)
        if(seq[i].node_id>0 && i!=srcIdx && (pkg_pos<0||i<pkg_pos))
            cands.push_back(i);
    if(cands.empty()){ rv.insert(rv.begin()+pick,pkg); return; } // rollback
    uniform_int_distribution<> cd(0,(int)cands.size()-1);
    seq[cands[cd(gen)]].resupply_vector.push_back(pkg);
}

// ---- Repair Sequence ----
void repairSequence(Chromosome& seq, const PDPData& data, mt19937& gen) {
    // --- node_id > 0: exactly once each ---
    map<int,int> cnt;
    for(auto& g:seq) if(g.node_id>0&&data.isCustomer(g.node_id)) cnt[g.node_id]++;
    vector<int> missing;
    for(int i=0;i<data.numNodes;i++) if(data.isCustomer(i)&&cnt[i]==0) missing.push_back(i);
    shuffle(missing.begin(),missing.end(),gen);
    int mi=0;
    for(auto& g:seq){
        if(g.node_id>0&&data.isCustomer(g.node_id)&&cnt[g.node_id]>1&&mi<(int)missing.size()){
            cnt[g.node_id]--; g.node_id=missing[mi++];
        }
    }
    for(;mi<(int)missing.size();mi++) seq.push_back(Gene(missing[mi]));

    // --- node_id == 0: exactly one separator ---
    vector<int> sepPos;
    for(int i=0;i<(int)seq.size();i++) if(seq[i].node_id==0) sepPos.push_back(i);
    if(sepPos.empty()){
        // Insert separator in middle
        int mid=max(1,(int)seq.size()/2);
        seq.insert(seq.begin()+mid,Gene(0));
    } else if(sepPos.size()>1){
        // Keep first, remove rest (from back to avoid index shift)
        for(int k=(int)sepPos.size()-1;k>=1;k--)
            seq.erase(seq.begin()+sepPos[k]);
    }
    // --- node_id == -1: leave untouched ---

    // --- P before DL precedence ---
    map<int,int> posP,posDL;
    for(int i=0;i<(int)seq.size();i++){
        if(seq[i].node_id<=0) continue;
        int id=seq[i].node_id;
        if(id>=(int)data.pairIds.size()) continue;
        int pid=data.pairIds[id];
        if(pid<=0) continue;
        if(data.nodeTypes[id]=="P")  posP[pid]=i;
        if(data.nodeTypes[id]=="DL") posDL[pid]=i;
    }
    for(auto& kv:posP){
        auto it=posDL.find(kv.first);
        if(it==posDL.end()) continue;
        if(kv.second>it->second) swap(seq[kv.second].node_id,seq[it->second].node_id);
    }
}

// ---- Selection ----
Chromosome tournamentSelection(const vector<Chromosome>& pop,
                               const vector<double>& fit,
                               int k, mt19937& gen) {
    uniform_int_distribution<> d(0,pop.size()-1);
    int best=d(gen);
    for(int i=1;i<k;i++){int idx=d(gen);if(fit[idx]<fit[best])best=idx;}
    return pop[best];
}

static bool evalImproved(const Chromosome& child,const Chromosome& p1,const Chromosome& p2){
    int n=child.size(),d1=0,d2=0;
    for(int i=0;i<n;i++){
        if(child[i].node_id!=p1[i].node_id) d1++;
        if(child[i].node_id!=p2[i].node_id) d2++;
    }
    int thr=max(1,n/5);
    return d1>=thr&&d2>=thr;
}
static bool mutImproved(const Chromosome& a,const Chromosome& b){
    if(a.size()!=b.size()) return true;
    for(int i=0;i<(int)a.size();i++) if(a[i].node_id!=b[i].node_id) return true;
    return false;
}

// ============================================================
// === MAIN GA ================================================
// ============================================================

PDPSolution geneticAlgorithmPDP(const PDPData& data, int popSize,
                                int maxGen, double mutRate, int runNum, bool /*small*/) {
    mt19937 rng(random_device{}() + (unsigned)runNum*12345u);
    cout << "\n=== GENETIC ALGORITHM + TABU (Gene-based) ===\n"
         << "Pop=" << popSize << " Gen=" << maxGen << " MutRate=" << mutRate << "\n";

    AdaptiveParams ap(mutRate);
    OnlineSurrogate sur;
    SolutionCache cache;

    // Init population (vector<Chromosome> from initStructuredPopulationPDP)
    vector<Chromosome> pop = initStructuredPopulationPDP(popSize, data, runNum);

    vector<double> fit(popSize);
    PDPSolution bestSol; bestSol.totalCost = numeric_limits<double>::infinity();
    Chromosome   bestSeq;

    for(int i=0;i<popSize;i++){
        auto s=evalChr(pop[i],data,cache);
        fit[i]=s.totalCost+s.totalPenalty;
        if(fit[i]<bestSol.totalCost+bestSol.totalPenalty){ bestSol=s; bestSeq=pop[i]; }
    }
    cout << "Init best: " << fixed << setprecision(2) << bestSol.totalCost
         << " (pen=" << bestSol.totalPenalty << ")\n";

    int noImpCtr=0, noImpEval=0;
    int tabuThr = data.numCustomers<=20 ? max(100,popSize*5) : max(200,popSize*20);
    int tabuRounds=0;
    int adaptInt=max(5,maxGen/20);

    for(int gen=0;gen<maxGen;gen++){
        // --- Crossover ---
        vector<Chromosome> offspring;
        vector<int>        cxTypes;
        vector<pair<double,double>> parentFit;

        auto chooseCrossover = [&]() -> int {
            const double eps = 0.05;
            vector<double> weights(4, eps);
            for (int i = 0; i < 4; ++i) {
                if (ap.crossoverUsage[i] > 0) {
                    double rate = ap.crossoverSuccess[i] / ap.crossoverUsage[i];
                    weights[i] = rate + eps;
                }
            }
            discrete_distribution<> pick(weights.begin(), weights.end());
            return pick(rng);
        };

        for(int i=0;i<popSize;i++){
            auto p1=tournamentSelection(pop,fit,3,rng);
            auto p2=tournamentSelection(pop,fit,3,rng);
            // Track parent fitness for proxy
            double f1=fit[0],f2=fit[0];
            for(int j=0;j<(int)pop.size();j++){
                bool m1=true,m2=true;
                for(int k=0;k<(int)pop[j].size()&&(m1||m2);k++){
                    if(pop[j][k].node_id!=p1[k].node_id) m1=false;
                    if(pop[j][k].node_id!=p2[k].node_id) m2=false;
                }
                if(m1) f1=fit[j];
                if(m2) f2=fit[j];
            }
            parentFit.push_back({f1,f2});

            int ct=chooseCrossover(); cxTypes.push_back(ct);
            Chromosome child;
            if(ct==0)      child=onePointCrossover(p1,p2,rng);
            else if(ct==1) child=orderCrossover(p1,p2,rng);
            else if(ct==2) child=pmxCrossover(p1,p2,rng);
            else           child=cycleCrossover(p1,p2,rng);
            repairSequence(child,data,rng);
            offspring.push_back(child);
        }

        // --- Mutation ---
        int mutCnt=(int)(offspring.size()*ap.currentMutationRate);
        uniform_int_distribution<> od(0,offspring.size()-1);
        for(int i=0;i<mutCnt;i++){
            int idx=od(rng);
            Chromosome orig=offspring[idx];
            int mt=ap.selectMutationType(rng);
            if(mt==0)      swapMutation(offspring[idx],rng);
            else if(mt==1) inversionMutation(offspring[idx],rng);
            else if(mt==2) scrambleMutation(offspring[idx],rng);
            else if(mt==3) insertionMutation(offspring[idx],rng);
            else if(mt==4) displacementMutation(offspring[idx],rng);
            else           droneResupplyMutation(offspring[idx],data,rng);
            repairSequence(offspring[idx],data,rng);
            ap.updateMutationSuccess(mt,mutImproved(offspring[idx],orig));
        }

        // --- Evaluate offspring (full fitness) ---
        vector<double> offit(offspring.size(), 0.0);
        int decoded = 0;
        for (int i = 0; i < (int)offspring.size(); ++i) {
            auto s = evalChr(offspring[i], data, cache);
            offit[i] = s.totalCost + s.totalPenalty;
            decoded++;

            double parent_best = (i < (int)parentFit.size())
                ? min(parentFit[i].first, parentFit[i].second)
                : numeric_limits<double>::infinity();
            ap.updateCrossoverSuccess(cxTypes[i], offit[i] < parent_best);
        }
        if(gen>0&&gen%20==0)
            cout << "[Gen " << gen << "] best=" << (bestSol.totalCost + bestSol.totalPenalty)
                 << " decoded=" << decoded << "/" << offspring.size() << "\n";

        // --- Selection: 50% best offspring, 20% random offspring, 30% best parents ---
        int nBO=(int)(popSize*0.5), nRO=(int)(popSize*0.2), nBP=popSize-nBO-nRO;
        vector<int> oi(offspring.size()); iota(oi.begin(),oi.end(),0);
        sort(oi.begin(),oi.end(),[&](int a,int b){return offit[a]<offit[b];});
        vector<int> pi(pop.size()); iota(pi.begin(),pi.end(),0);
        sort(pi.begin(),pi.end(),[&](int a,int b){return fit[a]<fit[b];});

        vector<Chromosome> newPop; vector<double> newFit;
        for(int i=0;i<nBO;i++){newPop.push_back(offspring[oi[i]]);newFit.push_back(offit[oi[i]]);}
        {   vector<int> rem; for(int i=nBO;i<(int)oi.size();i++) rem.push_back(oi[i]);
            shuffle(rem.begin(),rem.end(),rng);
            int ta=min(nRO,(int)rem.size());
            for(int i=0;i<ta;i++){newPop.push_back(offspring[rem[i]]);newFit.push_back(offit[rem[i]]);}
        }
        for(int i=0;i<nBP;i++){newPop.push_back(pop[pi[i]]);newFit.push_back(fit[pi[i]]);}
        pop=newPop; fit=newFit;

        // Update best
        if(fit[0]<bestSol.totalCost+bestSol.totalPenalty){
            auto s=evalChr(pop[0],data,cache);
            bestSol=s; bestSeq=pop[0];
            noImpCtr=0; noImpEval=0; ap.noImprovementCount=0;
            ap.currentMutationRate = ap.baseMutationRate;
            cout << "Gen " << gen << ": best=" << fixed << setprecision(2)
                 << bestSol.totalCost << " (pen=" << bestSol.totalPenalty << ")\n";
        } else {
            noImpCtr++; noImpEval+=decoded; ap.noImprovementCount++;
            if (ap.noImprovementCount > 50) {
                ap.currentMutationRate = min(0.8, ap.currentMutationRate * 1.05);
            }
        }

        // Adaptive rates
        if(gen>0&&gen%adaptInt==0){
            ap.adaptCrossoverRates(); ap.adaptMutationRates();
        }

        // --- Tabu phase ---
        if(noImpEval>=tabuThr){
            int topK=max(1,popSize/10);
            cout << "\n[TABU] " << noImpEval << " evals stagnant. Tabu on top " << topK << "\n";
            vector<int> si(popSize); iota(si.begin(),si.end(),0);
            sort(si.begin(),si.end(),[&](int a,int b){return fit[a]<fit[b];});
            double bestBefore=bestSol.totalCost+bestSol.totalPenalty;

            for(int k=0;k<topK;k++){
                int idx=si[k];
                Chromosome start=pop[idx];
                if(k>0) start=doubleBridgePerturbation(start,rng);
                repairSequence(start,data,rng);
                try {
                    Chromosome tabuRes=tabuSearchPDP(start,data,50,cache);
                    repairSequence(tabuRes,data,rng);
                    auto ts=evalChr(tabuRes,data,cache);
                    double tf=ts.totalCost+ts.totalPenalty;
                    if(tf<fit[idx]){
                        pop[idx]=tabuRes; fit[idx]=tf;
                        cout << "  [TABU] idx=" << idx << " -> " << tf << "\n";
                        if(tf<bestBefore){ bestSeq=tabuRes; bestSol=ts; bestBefore=tf; }
                    }
                } catch(const exception& e){ cerr << "Tabu err: " << e.what() << "\n"; }
            }

            noImpCtr=0; noImpEval=0; tabuRounds++;

            // Diversity restart after 3 stagnant tabu rounds
            if(tabuRounds>=3&&bestSol.totalCost+bestSol.totalPenalty>=bestBefore-0.01){
                cout << "[DIVERSITY] Restarting 80% of population\n";
                iota(si.begin(),si.end(),0);
                sort(si.begin(),si.end(),[&](int a,int b){return fit[a]<fit[b];});
                int keep=max(2,popSize/5);
                auto newInds=initStructuredPopulationPDP(popSize-keep,data,runNum+gen);
                int ri=0;
                for(int k=keep;k<popSize;k++){
                    pop[si[k]]= ri<(int)newInds.size()? newInds[ri++]
                              : doubleBridgePerturbation(bestSeq,rng);
                    repairSequence(pop[si[k]],data,rng);
                    auto s=evalChr(pop[si[k]],data,cache);
                    fit[si[k]]=s.totalCost+s.totalPenalty;
                }
                tabuRounds=0;
            }
        }
    }

    cout << "\n=== GA DONE ===\n"
         << "Final best: " << fixed << setprecision(2) << bestSol.totalCost
         << " (pen=" << bestSol.totalPenalty << ")\n";
    return bestSol;
}

