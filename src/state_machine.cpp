#include <geranos_pole_trajectory/state_machine.h>

namespace geranos_planner {

	std::map<State, State> StateTransitions = {
		{State::GET_WHITE, State::PLACE_WHITE},
		{State::PLACE_WHITE, State::GET_GREY},
		{State::GET_GREY, State::PLACE_GREY},
		{State::PLACE_GREY, State::DONE}
	};

	TrajectoryState::TrajectoryState() {
		currentState = State::GET_WHITE;
	}
	void TrajectoryState::toggle() {
		currentState = StateTransitions[currentState];
	}
	std::string TrajectoryState::getCurrMode() {
		switch(currentState) {
			case State::GET_WHITE:
		        return "GET_WHITE";
			case State::PLACE_WHITE:
				return "PLACE_WHITE";
			case State::GET_GREY:
				return "GET_GREY";
			case State::PLACE_GREY:
				return "PLACE_GREY";
			case State::DONE:
				return "DONE";
			default:
				return "";
		}
	}
	void TrajectoryState::reset() {
		currentState = State::GET_WHITE;
	}

}