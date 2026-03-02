function gen = scenario_factory(scenario_type)
    gen = struct();
    switch lower(string(scenario_type))
        case "one_tx_one_rx_static"
            gen.generate = @scenarios.one_tx_one_rx_static;
        case "one_tx_two_rx_static_offset"
            gen.generate = @scenarios.one_tx_two_rx_static_offset;
        case "four_tx_two_rx_static_dd_wls"
            gen.generate = @scenarios.four_tx_two_rx_static_dd_wls;
        case "four_tx_two_rx_dynamic_linear"
            gen.generate = @scenarios.four_tx_two_rx_dynamic_linear;
        otherwise
            error('Unknown scenario_type: %s', scenario_type);
    end
end
