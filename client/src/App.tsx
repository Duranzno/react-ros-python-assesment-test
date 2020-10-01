
import React from 'react';
import { ThemeProvider } from '@material-ui/styles';
import { CssBaseline } from '@material-ui/core';
import { theme } from './theme';
import { AppProvider, useAppContext } from './providers';
import { NotConnected } from './components';
import { RosContainer, AppContainer } from './containers';
import { useRos } from './hooks';

import './App.css';

const App: React.FC = () => {
    const { state: { isConnected } } = useAppContext();
    const { connect, ros } = useRos()
    return (
        <AppContainer>
            {
                (isConnected && ros)
                    ? <RosContainer ros={ros} />
                    : <NotConnected onRefresh={connect} />
            }
        </AppContainer>
    );
}
function ProviderWrapper(): React.ReactElement {
    return (
        <AppProvider>
            <ThemeProvider theme={theme}>
                <CssBaseline />
                <App />
            </ThemeProvider>
        </AppProvider>
    );
}
export default ProviderWrapper;
