function read_gcode(filename)
    fid = fopen(filename, 'r');
    while ~feof(fid)
        line = fgetl(fid);
        process_gcode(line);
    end
    fclose(fid);
end

function process_gcode(line)
    tokens = regexp(line, '([A-Z])([-+]?[0-9]*\.?[0-9]+)', 'tokens');
    for i = 1:length(tokens)
        command = tokens{i}{1};
        value = str2double(tokens{i}{2});
        fprintf('Command: %s, Value: %.3f\n', command, value);
    end
end
