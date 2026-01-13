function [Score] = calMetric(MetricIndex, PopObj, problemIndex, M, hvSamples)
    % calMetric: Calculate performance metrics
    % MetricIndex: 1 for Hypervolume (HV), 2 for Pure Diversity (PD)
    % M: Number of objectives
    
    if nargin < 4, M = size(PopObj, 2); end
    
    if isempty(PopObj)
        Score = 0;
        return;
    end
    
    if MetricIndex == 1
        % Reference point for HV calculation
        if M == 2
            if problemIndex == 1
                refPoint = [4000, 4000];
            elseif problemIndex == 2
                refPoint = [50, 10];
            else
                refPoint = [4000, 4000];
            end
        elseif M == 4
            % Reference point for [Length, Threat, Altitude, Smoothness]
            % These values should be adjusted based on the objective ranges
            refPoint = [4000, 1000, 4000, 100000];
        else
            refPoint = ones(1, M) * 10000;
        end
        if nargin >= 5 && ~isempty(hvSamples)
            Score = HV(PopObj, refPoint, hvSamples);
        else
            Score = HV(PopObj, refPoint);
        end
    else
        Score = PD(PopObj);
    end
end
