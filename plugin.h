// Evoplex <https://evoplex.org>

#ifndef FOLLOWFLEE_H
#define FOLLOWFLEE_H

#include <map>
#include <plugininterface.h>

namespace evoplex {
class FollowFlee: public AbstractModel
{
public:
    /**
     * @brief Initializes the plugin.
     * This method is called when the plugin is created and
     * is used to get the user inputs.
     * @return true if successful
     */
    bool init() override;

    /**
     * @brief It is executed before the algorithmStep() loop.
     */
    void beforeLoop() override;

    /**
     * @brief It is executed in a loop and contains all the logic to perform ONE step.
     * @returns true if algorithm is good for another step or false to stop asap.
     */
    bool algorithmStep() override;

private:
    /**
     * The node's attributes as defined in the metadata.json
     */
    enum NodeAttrs { Strategy, Actions, Score };

    /**
     * The replacement modes implemented in the model (metadata.json)
     */
    enum RepMode { SimpleBD, NeighbourBD };

    /**
     * A convenient struct used to calculate and determine the move performed by an agent.
     */
    struct FreeCell {
        int id;
        int score;
    };

    /**
     * A convenient struct used to hold the neighbourhood state of an agent.
     */
    struct Horizon {
        std::vector<Node> cooperators;    // the cooperators around
        std::vector<Node> defectors;      // the cooperators around
        std::vector<FreeCell> freeCells;  // the free cells around

        Horizon(quint32 size) {
            // preallocate enough memory (optimization)
            cooperators.reserve(size);
            defectors.reserve(size);
            freeCells.reserve(size + 1); // +1 to include the agent itself
        }

        void clear() {
            cooperators.clear();
            defectors.clear();
            freeCells.clear();
        }
    };

    /**
     * Update the score of a given agent, also keeping track of the
     * neighbourhood state, i.e., cooperators, defectors and free cells around.
     */
    void updateScoreAndHorizon(Node& agent, Horizon& horizon) const;

    /**
     * Update the position of a given agent based on its neighbourhood state (horizon)
     */
    void updatePosition(Node& agent, Horizon& horizon);

    /**
     * Replacement strategy: replace the worst X agents by the best X agents
     */
    void simpleBD(quint32 agentsToReplace);

    /**
     * Replacement strategy: replace the worst X agents by the best X agents
     * but trying to keep the offspring in the parent neighbourhood
     */
    void neighbourBD(quint32 agentsToReplace);

    /**
     * Play the prisoner's dilemma game
     */
    int playGame(int strA, int strB) const;

    /**
     * Move the @p agent to the @p targetId
     */
    void move(Node& agent, int targetId);

    /**
     * Choose an empty cell at random
     */
    Node selectEmptyCell() const;

    /**
     * Copy attributes from the agent @p src to the agent @p tgt
     */
    void copyAttrs(Node& src, Node& tgt) const;

    /**
     * Sets all the agent's attrs to zero
     */
    void clearAttrs(Node& agent);

    /**
     * Evaluate the free cells in the neighbourhood
     */
    void evalFreeCells(std::vector<FreeCell>& freeCells,
            const std::vector<Node>& neighbours, quint8 action) const;

    /**
     * The center cell (0) sums zero and the others subtract one
     */
    void stayStill(std::vector<FreeCell>& freeCells, int numNeighbours) const;

    /**
     * The intersecting neighbours sum one and the others sum zero
     */
    void follow(std::vector<FreeCell>& freeCells, const Node& neighbour) const;

    /**
     * The intersecting neighbours sum zero and the others sum one
     */
    void flee(std::vector<FreeCell>& freeCells, const Node& neighbour) const;

    /**
     * All neighbours sum randomly (ie, -1, 0 or +1 for each neighbour)
     */
    void random(std::vector<FreeCell>& freeCells, int numNeighbours) const;

    /**
     * Sort a vector of agents by score (descending)
     */
    void sortAgentsByScore(std::vector<Node> agents) const;

    /**
     * An auxiliary function to convert a string to RepMode
     */
    RepMode repModeFromString(const QString& s);

    // the model attributes (as defined in the metadata.json)
    RepMode m_repMode;  // replacement mode
    double m_repRate;   // replacement rate
    int m_stepsPerGen;

    std::vector<Node> m_agents; // the cells with live agents, ie, strategy=[1,2]
    std::map<int, Node> m_emptyCells; // the empty cells


};
} // evoplex
#endif // FOLLOWFLEE_H
