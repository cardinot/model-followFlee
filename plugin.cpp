// Evoplex <https://evoplex.org>

#include <bitset>

#include "plugin.h"

namespace evoplex {

bool FollowFlee::init()
{
    m_repMode = repModeFromString(attr("repMode", "").toString());
    m_repRate = attr("repRate", -1.0).toDouble();
    m_stepsPerGen = attr("stepsPerGen", -1).toInt();

    return m_repRate > -1 && m_stepsPerGen > -1;
}

void FollowFlee::beforeLoop()
{
    m_agents.clear();
    m_emptyCells.clear();
    m_agents.reserve(nodes().size());

    // Find the non-empty nodes (agents)
    for (Node node : nodes()) {
        if (node.attr(Strategy).toInt() > 0) {
            m_agents.emplace_back(node);
        } else {
            m_emptyCells.insert({node.id(), node});
        }
    }
}

bool FollowFlee::algorithmStep()
{
    if (m_agents.empty()) {
        return true; // nothing to do
    }

    // sort agents by id
    // it's important to ensure the same initial condition before shuffling
    // otherwise, the play and step-by-step buttons will lead to different outputs
    std::sort(m_agents.begin(), m_agents.end(),
        [](Node i,Node j) { return i.id() <  j.id(); });

    // shuffle the vector of ids
    Utils::shuffle(m_agents, prg());

    // A convenient struct to hold the neighbourhood state.
    // As it's a regular graph, let's create it only once, reserve enough
    // space and reuse the same object (clear) when necessary.
    Horizon horizon(graph()->attr("neighbours").toUInt());

    // for each agent in the population
    for (Node& agent : m_agents) {
        // reset score
        agent.setAttr(Score, 0);

        // the agent takes s steps per generation
        for (int step = 0; step < m_stepsPerGen; ++step) {
            updateScoreAndHorizon(agent, horizon);
            updatePosition(agent, horizon);
        }
    }

    // replacement phase; prepares the next generation
    auto agentsToReplace = static_cast<quint32>(floor(m_agents.size() * m_repRate));
    if (agentsToReplace > 0) {
        if (m_repMode == SimpleBD) {
            simpleBD(agentsToReplace);
        } else if (m_repMode == NeighbourBD) {
            neighbourBD(agentsToReplace);
        } else {
            qFatal("the replacement mode is invalid!");
        }
    }

    return true;
}

void FollowFlee::updateScoreAndHorizon(Node& agent, Horizon& horizon) const
{
    horizon.clear();

    // the agent can stay still; so, it's a free cell too!
    // important: the center cell is always the first!
    horizon.freeCells.push_back({agent.id(), 0});

    const int strA = agent.attr(Strategy).toInt();
    int score = agent.attr(Score).toInt();
    for (Node neighbour : agent.outEdges()) {
        int strB = neighbour.attr(Strategy).toInt();

        // this cell is empty
        if (strB == 0) {
            horizon.freeCells.push_back({neighbour.id(), 0});
            continue;
        }

        // accumulate the score received by playing the
        // prisoner's dilemma game with all neighbours
        score += playGame(strA, strB);

        // keep track of the neighbourhood state
        if (strB == 1) {
            horizon.cooperators.emplace_back(neighbour);
        } else {
            horizon.defectors.emplace_back(neighbour);
        }
    }

    // update the agent's score
    agent.setAttr(Score, score);
}

void FollowFlee::updatePosition(Node& agent, Horizon& horizon)
{
    Q_ASSERT_X(horizon.freeCells.size() > 0, "updatePosition",
        "freeCells counts the agent itself, so the size is always >0");

    if (horizon.freeCells.size() == 1) {
        return; // no place to go!
    }

    size_t numNeighbours = agent.outEdges().size() - (horizon.freeCells.size() - 1);

    // no neighbours? move at random!
    if (numNeighbours == 0) {
        move(agent, horizon.freeCells.at(prg()->uniform(horizon.freeCells.size()-1)).id);
        return;
    }

    // convert decimal to 8-bit
    // important! in a bitset, the order positions are counted from right to left
    const std::bitset<8> actions(agent.attr(Actions).toUInt());

    // evaluate the free cells based on the neighbourhood state
    if (numNeighbours == horizon.cooperators.size()) { // only cooperators
        evalFreeCells(horizon.freeCells, horizon.cooperators,
                      actions[7] * 2 + actions[6]);
    } else if (numNeighbours == horizon.defectors.size()) { // only defectors
        evalFreeCells(horizon.freeCells, horizon.defectors,
                      actions[5] * 2 + actions[4]);
    } else { // cooperators and defectors
        evalFreeCells(horizon.freeCells, horizon.cooperators,
                      actions[3] * 2 + actions[2]);
        evalFreeCells(horizon.freeCells, horizon.defectors,
                      actions[1] * 2 + actions[0]);
    }

    // pick the free cells with the highest score
    int highestScore = INT32_MIN;
    std::vector<int> highestScoreIds;
    highestScoreIds.reserve(horizon.freeCells.size());
    for (auto fc : horizon.freeCells) {
        if (fc.score > highestScore) {
            highestScore = fc.score;
            highestScoreIds.clear();
            highestScoreIds.emplace_back(fc.id);
        } else if (fc.score == highestScore) {
            highestScoreIds.emplace_back(fc.id);
        }
    }

    // finally, set the position!
    Q_ASSERT(highestScoreIds.size() > 0);
    if (highestScoreIds.size() == 1) {
        move(agent, highestScoreIds.front());
    } else {
        move(agent, highestScoreIds.at(prg()->uniform(highestScoreIds.size()-1)));
    }
}

void FollowFlee::simpleBD(quint32 agentsToReplace)
{
    sortAgentsByScore(m_agents);

    // make the worst X cells available
    const size_t last = m_agents.size() - 1;
    for (quint32 i = 0; i < agentsToReplace; ++i) {
        Node n = m_agents.at(last-i);
        m_emptyCells.insert({n.id(), n});
    }

    // now we copy the best X agents and place them randomly on the grid
    for (quint32 i = 0; i < agentsToReplace; ++i) {
        // choose an empty cell at random
        Node tgt = selectEmptyCell();

        // make this cell active
        m_emptyCells.erase(tgt.id());
        m_agents.emplace_back(tgt);
        copyAttrs(m_agents.at(i), m_agents.back());
    }

    // fix containers
    m_agents.erase(m_agents.end()-2*agentsToReplace, m_agents.end()-agentsToReplace);
    for (Node e : m_emptyCells) {
        clearAttrs(e);
    }
}

void FollowFlee::neighbourBD(quint32 agentsToReplace)
{
    sortAgentsByScore(m_agents);

    // make the worst X cells available
    const size_t last = m_agents.size() - 1;
    for (quint32 i = 0; i < agentsToReplace; ++i) {
        Node n = m_agents.at(last-i);
        m_emptyCells.insert({n.id(), n});
    }

    std::vector<Node> freeCells;
    freeCells.reserve(graph()->attr("neighbours").toUInt());

    // now we copy the best X agents and place the copies randomly around the parent
    for (quint32 i = 0; i < agentsToReplace; ++i) {
        Node parent = m_agents.at(i);

        // checks if the parent has free cells around
        freeCells.clear();
        for (Node neighbour : parent.outEdges()) {
            if (neighbour.attr(Strategy).toInt() == 0)
                freeCells.emplace_back(neighbour);
        }

        Node tgt;
        if (freeCells.empty()) { // no space
            tgt = selectEmptyCell(); // random
        } else {
            tgt = freeCells.at(prg()->uniform(freeCells.size()-1));
        }

        // make this cell active
        m_emptyCells.erase(tgt.id());
        m_agents.emplace_back(tgt);
        copyAttrs(parent, m_agents.back());
    }

    // fix containers
    m_agents.erase(m_agents.end()-2*agentsToReplace, m_agents.end()-agentsToReplace);
    for (Node e : m_emptyCells) {
        clearAttrs(e);
    }
}

int FollowFlee::playGame(int strA, int strB) const
{
    switch ((strA-1)*2 + (strB-1)) {
        case 0: // CC : Reward for mutual cooperation
            return 3;
        case 1: // CD : Sucker's payoff
            return 0;
        case 2: // DC : Temptation to defect
            return 5;
        case 3: // DD : Punishment for mutual defection
            return 1;
        default: // it should never happen
            qFatal("Error! Invalid strategies (%d,%d)", strA, strB);
    }
}

void FollowFlee::move(Node& agent, int targetId)
{
    if (agent.id() != targetId) {
        Node tgt = node(targetId);
        m_emptyCells.erase(tgt.id());
        copyAttrs(agent, tgt);
        clearAttrs(agent);
        m_emptyCells.insert({agent.id(), agent});
        agent = tgt;
    }
}

Node FollowFlee::selectEmptyCell() const
{
    size_t itPos = prg()->uniform(m_emptyCells.size()-1);
    return std::next(m_emptyCells.cbegin(), itPos)->second;
}

void FollowFlee::copyAttrs(Node& src, Node& tgt) const
{
    tgt.setAttr(Strategy, src.attr(Strategy));
    tgt.setAttr(Actions, src.attr(Actions));
    tgt.setAttr(Score, src.attr(Score));
}

void FollowFlee::clearAttrs(Node& agent)
{
    agent.setAttr(Strategy, 0);
    agent.setAttr(Actions, 0);
    agent.setAttr(Score, 0);
}

void FollowFlee::evalFreeCells(std::vector<FreeCell>& freeCells,
        const std::vector<Node>& neighbours, quint8 action) const
{
    switch (action) {
    case 0:
        stayStill(freeCells, static_cast<int>(neighbours.size()));
        return;
    case 1:
        for (const Node& n : neighbours) follow(freeCells, n);
        return;
    case 2:
        for (const Node& n : neighbours) flee(freeCells, n);
        return;
    case 3:
        random(freeCells, static_cast<int>(neighbours.size()));
        return;
    default:
         qFatal("Error! Invalid action (%d)", action);
    }
}

void FollowFlee::stayStill(std::vector<FreeCell>& freeCells, int numNeighbours) const
{
    // the center cell (0) sums zero and the others subtract one
    for (size_t i = 1; i < freeCells.size(); ++i) {
        freeCells.at(i).score -= numNeighbours;
    }
}

void FollowFlee::follow(std::vector<FreeCell>& freeCells, const Node& neighbour) const
{
    // the intersecting neighbours sum one and the others sum zero
    for (auto& fc : freeCells) {
        for (const Node& n : neighbour.outEdges()) {
            if (fc.id == n.id()) {
                fc.score += 1;
                break;
            }
        }
    }
}

void FollowFlee::flee(std::vector<FreeCell>& freeCells, const Node& neighbour) const
{
    // the intersecting neighbours sum zero and the others sum one
    for (auto& fc : freeCells) {
        bool intersects = false;
        for (const Node& n : neighbour.outEdges()) {
            if (fc.id == n.id()) {
                intersects = true;
                break;
            }
        }
        if (!intersects) fc.score += 1;
    }
}

void FollowFlee::random(std::vector<FreeCell>& freeCells, int numNeighbours) const
{
    // all neighbours sum randomly (ie, -1, 0 or +1 for each neighbour)
    for (auto& fc : freeCells) {
        fc.score += prg()->uniform(-numNeighbours, numNeighbours);
    }
}

void FollowFlee::sortAgentsByScore(std::vector<Node> agents) const
{
    std::sort(agents.begin(), agents.end(),
        [](Node i,Node j) {
            return i.attr(Score).toInt() >  j.attr(Score).toInt();
        });
}

FollowFlee::RepMode FollowFlee::repModeFromString(const QString& s)
{
    if (s == "simpleBD") return SimpleBD;
    if (s == "neighbourBD") return NeighbourBD;
    qFatal("the replacement mode is invalid!");
}

} // evoplex
REGISTER_PLUGIN(FollowFlee)
#include "plugin.moc"
