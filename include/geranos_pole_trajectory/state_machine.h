#pragma once
#include <map>
#include <string>

namespace geranos_planner {
	enum State 
	{
		GET_WHITE,
		PLACE_WHITE,
		GET_GREY,
		PLACE_GREY,
		DONE
	};

	class TrajectoryState
	{
	public:
		TrajectoryState();
		void toggle();
		std::string getCurrMode();
		void reset();
		inline State currState() const { return currentState; }

	private:
		State currentState;
	};
}

