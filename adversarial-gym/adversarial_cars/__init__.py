from gym.envs.registration import register

register(
    id='adversarial_cars-v0',
    entry_point='adversarial_cars.envs:AdversarialCars',
)