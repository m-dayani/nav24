//
// Created by masoud on 6/10/25.
// Convert ORB_Voc.txt (old ORB-SLAM version) to ORB_Voc.yml.gz (new version)

#include <iostream>
#include <boost/filesystem.hpp>
#include <DBoW2/DBoW2.h>


using namespace std;


class ORBVocabulary : public OrbVocabulary {
public:
    bool loadFromTextFile(const std::string &filename)
    {
        ifstream f;
        f.open(filename.c_str());

        if(f.eof())
            return false;

        m_words.clear();
        m_nodes.clear();

        string s;
        getline(f,s);
        stringstream ss;
        ss << s;
        ss >> m_k;
        ss >> m_L;
        int n1, n2;
        ss >> n1;
        ss >> n2;

        if(m_k<0 || m_k>20 || m_L<1 || m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
        {
            std::cerr << "Vocabulary loading failure: This is not a correct text file!" << endl;
            return false;
        }

        m_scoring = (DBoW2::ScoringType)n1;
        m_weighting = (DBoW2::WeightingType)n2;
        createScoringObject();

        // nodes
        int expected_nodes =
                (int)((pow((double)m_k, (double)m_L + 1) - 1)/(m_k - 1));
        m_nodes.reserve(expected_nodes);

        m_words.reserve(pow((double)m_k, (double)m_L + 1));

        m_nodes.resize(1);
        m_nodes[0].id = 0;
        while(!f.eof())
        {
            string snode;
            getline(f,snode);
            stringstream ssnode;
            ssnode << snode;

            int nid = m_nodes.size();
            m_nodes.resize(m_nodes.size()+1);
            m_nodes[nid].id = nid;

            int pid ;
            ssnode >> pid;
            m_nodes[nid].parent = pid;
            m_nodes[pid].children.push_back(nid);

            int nIsLeaf;
            ssnode >> nIsLeaf;

            stringstream ssd;
            for(int iD=0;iD<DBoW2::FORB::L;iD++)
            {
                string sElement;
                ssnode >> sElement;
                ssd << sElement << " ";
            }
            DBoW2::FORB::fromString(m_nodes[nid].descriptor, ssd.str());

            ssnode >> m_nodes[nid].weight;

            if(nIsLeaf>0)
            {
                int wid = m_words.size();
                m_words.resize(wid+1);

                m_nodes[nid].word_id = wid;
                m_words[wid] = &m_nodes[nid];
            }
            else
            {
                m_nodes[nid].children.reserve(m_k);
            }
        }

        return true;
    }
};

int main(int argc, char** argv) {

    if (argc < 2) {
        cout << "Usage: " << argv[0] << " path_old_voc.txt [path_new_voc.yml.gz]\n";
        return 1;
    }

    string pathVoc = argv[1];
    boost::filesystem::path pVoc(pathVoc);
    if (pathVoc.empty() || !exists(pVoc)) {
        cout << "path: '" << pathVoc << "' is not valid, abort!\n";
        return 2;
    }

    string pathOut = pathVoc.substr(0, pathVoc.size() - 4) + ".yml.gz";
    if (argc >= 3) {
        pathOut = argv[2];
    }

    cout << "Saving converted voc to: " << pathOut << "\n";

    auto mpOrbVoc = make_shared<ORBVocabulary>();
    bool res = mpOrbVoc->loadFromTextFile(pathVoc);
    if (!res) {
        cout << "Unfortunately, couldn't load ORB Voc\n";
        return 3;
    }

    mpOrbVoc->save(pathOut);

    cout << "Finished conversion, press a key to exit...\n";
    getchar();

    return 0;
}

