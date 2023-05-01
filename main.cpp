//
// Created by zartris on 4/9/23.
//

# include "bhs_game/models/AirportLogistic.hpp"

// create main function
int main() {
    printf("The main function is running \n");
    bhs_game::AirportLogistic model(0, false);

    model.run(10);

    printf("The main function is done \n");
    return 0;
}