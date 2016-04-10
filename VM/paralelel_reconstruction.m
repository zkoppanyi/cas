function [params, bins_res, cloud_res] = paralelel_reconstruction(model_cloud, params, params_baund, n_iter, n_neighb, settings)
    settings.n_neighb         = n_neighb;
    results = zeros(n_iter,5);
    
    if strcmp(settings.model, 'CV2DoF')
        results = zeros(n_iter,3);
    end;
        
    parfor i = 1 : n_iter,
        [params_sol, bins] = mexEntropyMinimization(model_cloud, params, params_baund, settings)
        results(i, :) = [bins, params];
    end;
    [bins_res, midx] = min(results(:,1));
    params_sol = results(midx, 2:end);
    %settings.model           = 'CA2DoF';
    [~, cloud_res] = mexReconstruct(model_cloud, params_sol, settings);