function output_of_saturation = my_saturation(value_to_be_checked, low_limit, high_limit)
    output_of_saturation = min(high_limit, max(low_limit, value_to_be_checked));
end

